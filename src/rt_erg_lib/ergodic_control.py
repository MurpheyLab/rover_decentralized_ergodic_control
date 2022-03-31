#!/usr/bin/env/ python
import rospy
import numpy as np

from replay_buffer import ReplayBuffer
from basis import Basis
from barrier import Barrier
from target_dist import TargetDist

from std_msgs.msg import String, Float32MultiArray, Bool

from collections import defaultdict

import matplotlib.pyplot as plt
import os

import datetime
curr_date_time = datetime.datetime.now()
cmonth = curr_date_time.strftime("%b")
cday = curr_date_time.strftime("%d")
chour = curr_date_time.strftime("%H")
cmin = curr_date_time.strftime("%M")
csec = curr_date_time.strftime("%S")
cyear = curr_date_time.strftime("%Y")

# original capacity=100000; horizon=100; num_basis is set in agent.py, weights is the "R" matrix
class RTErgodicControl(object):
    def __init__(self, agent_name, model,
                 weights=None, horizon=15, num_basis=15,
                 capacity=1000, batch_size=100): #batch_size is set in agent.py; capacity is set here in this file.

        rospy.logwarn('Creating ergodic controller for %s', agent_name)

        self.agent_name     = agent_name
        self.model          = model
        self.horizon        = horizon
        self.replay_buffer  = ReplayBuffer(capacity)
        self.batch_size     = batch_size

        self.basis = Basis(self.model.explr_space, num_basis=num_basis)
        self.lamk  = np.exp(-0.8*np.linalg.norm(self.basis.k, axis=1))
        self.barr  = Barrier(self.model.explr_space)

        self.target_dist = TargetDist(basis=self.basis)
        print("created a TargetDist")

        # FX3 version: publisher and subscriber for passing ck values amongst agents
        #rospy.Subscriber('received_floats', Float32MultiArray, self.ck_comms_callback)
        #self._ck_pub = rospy.Publisher('send_floats', Float32MultiArray)

        # the publisher and subscriber for passing ck values amongst agents via CCAST; 1 = STOMP, 2 = Tablet/Tanvas
        rospy.Subscriber('OUTPUT_TOPIC_1', String, self.ck_comms_callback)
        self._ck_pub = rospy.Publisher('INPUT_TOPIC_1', String, queue_size=1)

        rospy.Subscriber('OUTPUT_TOPIC_2', String, self.ck_comms_callback2)
        self._ck_pub2 = rospy.Publisher('INPUT_TOPIC_2', String, queue_size=1)

        # Are we using the tablet/tanvas?
        self.tanvas = False
        rospy.Subscriber('tanvas_status', Bool, self.tanvas_status_callback)

        self.u_seq = [0.0*np.zeros(self.model.action_space.shape[0])
                        for _ in range(horizon)]
        if weights is None:
            weights = {'R' : np.eye(self.model.action_space.shape[0])}
        self.Rinv = np.linalg.inv(weights['R'])

        self._phik  = None
        self.ck     = None

        #self._ck_dict = {}
        self._ck_dict = defaultdict(list)

        # if a ck value from a particular agent was last received > dropout_protection_time; don't use that agent's ck in the reconstruction
        self.dropout_protection_time = rospy.get_param("/control_node/dropout_protection_time")

        if (self.agent_name == "rover_0"):
            self.init_tablet_fig()

        ## create the plotting directory
        real_path=os.path.realpath(__file__)
        base_path = os.path.dirname(real_path)
        self.plotting_dir = base_path+"/data/{}_{}_{}_{}_{}_{}_{}_ergodic_metric_plots".format(cyear, cmonth, cday, chour, cmin, csec, self.agent_name)
        print("ergodic metric plotting_dir is: {}".format(self.plotting_dir))
        try:
            os.makedirs(self.plotting_dir)
        except:
            print("couldn't make plotting directory")
        self.erg_ts = []
        self.erg_metric_vals = []
        self.norm_erg_metric_vals = []

        self.start_time = rospy.get_time()
        self.loop_counter = 0

        ## storing the max ergodic metric for each specification (reset when a new one comes in)
        self.max_ergodic_metric = 1.0

    def init_tablet_fig(self):
        self.fig = None
        ## plotting both the normalized and unnormalized ergodic metric
        # self.fig, self.ax = plt.subplots(2,1)
        # self.ax[0].set_title("Ergodic Metric vs. Time")
        # self.ax[0].set_xlabel("Time [s]")
        # self.ax[0].set_ylabel("Ergodic Metric")
        # self.ax[1].set_title("Normalized Ergodic Metric vs. Time")
        # self.ax[1].set_xlabel("Time [s]")
        # self.ax[1].set_ylabel("Normalized Ergodic Metric")
        # self.fig.tight_layout()

        ## plotting just the normalized ergodic metric
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Normalized Ergodic Metric vs. Time")
        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Normalized Ergodic Metric")
        self.fig.tight_layout()
        
    # FX3 version: Updates the reconstruction of the trajectory
    # def ck_comms_callback(self, msg):
    #     if int(msg.data[0]) != self.agent_name:
    #         if self._ck_dict.has_key(int(msg.data[0])):
    #             self._ck_dict[int(msg.data[0])] = np.array(msg.data[1:])
    #         else:
    #             self._ck_dict.update({int(msg.data[0]) : np.array(msg.data[1:])})


    # FX4 and beyond version: Updates the reconstruction of the trajectory.
    def ck_comms_callback(self, msg): #for receiving targets from STOMP
        if (not self.tanvas):
            msg_str = msg.data
            msg_str_to_list = list(msg_str.split(" "))
            # rospy.logwarn("msg_str_to_list[0] value: %s", msg_str_to_list[0])

            # get the timestamp indicating when this ck value was received
            ck_time_received = rospy.get_time()

            if msg_str_to_list[0] != self.agent_name:
                if self._ck_dict.has_key(msg_str_to_list[0]):
                    # rospy.logwarn("_ck_dict already has key %s", msg_str_to_list[0])
                    self._ck_dict[msg_str_to_list[0]][0] = np.array(msg_str_to_list[1:], dtype=np.float32)
                    self._ck_dict[msg_str_to_list[0]][1] = ck_time_received

                else:
                    # rospy.logwarn("_ck_dict does not have key %s", msg_str_to_list[0])
                    self._ck_dict.update({msg_str_to_list[0] : [np.array(msg_str_to_list[1:], dtype=np.float32), ck_time_received]})

    def ck_comms_callback2(self, msg): #for receiving targets from Tablet/Tanvas
        if (self.tanvas):
            msg_str = msg.data
            msg_str_to_list = list(msg_str.split(" "))
            # rospy.logwarn("msg_str_to_list[0] value: %s", msg_str_to_list[0])

            # get the timestamp indicating when this ck value was received
            ck_time_received = rospy.get_time()

            if msg_str_to_list[0] != self.agent_name:
                if self._ck_dict.has_key(msg_str_to_list[0]):
                    # rospy.logwarn("_ck_dict already has key %s", msg_str_to_list[0])
                    self._ck_dict[msg_str_to_list[0]][0] = np.array(msg_str_to_list[1:], dtype=np.float32)
                    self._ck_dict[msg_str_to_list[0]][1] = ck_time_received

                else:
                    # rospy.logwarn("_ck_dict does not have key %s", msg_str_to_list[0])
                    self._ck_dict.update({msg_str_to_list[0] : [np.array(msg_str_to_list[1:], dtype=np.float32), ck_time_received]})


    def tanvas_status_callback(self, msg):
        self.tanvas = msg.data

    def reset(self):
        self.u_seq = [0.0*np.zeros(self.model.action_space.shape[0])
                for _ in range(self.horizon)]
        self.replay_buffer.reset()

    def __call__(self, state, ck_list=None, agent_num=None):
        assert self.target_dist.phik is not None, 'Forgot to set phik, use set_target_phik method'

        if self.target_dist.has_update == True:
            self.replay_buffer.reset()
            self.target_dist.has_update = False
            # reset the max ergodic metric
            self.max_ergodic_metric = 1.0

        self.u_seq[:-1] = self.u_seq[1:]
        self.u_seq[-1]  = np.zeros(self.model.action_space.shape[0])

        x = self.model.reset(state.copy())

        pred_traj = []
        dfk       = []
        fdx       = []
        fdu       = []
        dbar      = []
        for t in range(self.horizon):
            # collect all the information that is needed
            pred_traj.append(
                x[self.model.explr_idx]
            )
            dfk.append(
                self.basis.dfk(x[self.model.explr_idx])
            )
            fdx.append(self.model.fdx(x, self.u_seq[t]))
            fdu.append(self.model.fdu(x))
            dbar.append(
                self.barr.dx(x[self.model.explr_idx])
            )

            # step the model forwards
            # x = self.model.step(self.u_seq[t])
            x = self.model.step(self.u_seq[t] * 0.)


        # sample any past experiences to create a representation of the trajectory
        if len(self.replay_buffer) > self.batch_size:
            past_states = self.replay_buffer.sample(self.batch_size)
            pred_traj   = pred_traj + past_states
        else:
            past_states = self.replay_buffer.sample(len(self.replay_buffer))
            pred_traj   = pred_traj + past_states

        # calculate the cks for the trajectory *** this is also in the utils file
        N = len(pred_traj)
        ck = np.sum([self.basis.fk(xt) for xt in pred_traj], axis=0) / N
        self.ck = ck.copy()

        # publish Ck as a String
        ck_msg = String()
        _send_arr = []
        _send_arr.append(self.agent_name)
        ck_msg_list = ck.copy().tolist()
        for val in ck_msg_list:
            _send_arr.append(val)
        _send_arr = ' '.join([str(elem) for elem in _send_arr])
        ck_msg.data = _send_arr

        # Agents receiving targets from STOMP should not share ck's with agents receiving targets from the tablet (and vice-versa)
        #self._ck_pub.publish(ck_msg)
        if (not self.tanvas):
            self._ck_pub.publish(ck_msg)
        else:
            self._ck_pub2.publish(ck_msg)

        if len(self._ck_dict.keys()) > 1:
            #rospy.logwarn("_ck_dict has more than 1 key")

            curr_time = rospy.get_time()
            #rospy.logwarn("curr_time: %f", curr_time)

            if self._ck_dict.has_key(self.agent_name):
                self._ck_dict[self.agent_name][0] = ck
                self._ck_dict[self.agent_name][1] = curr_time
            else:
                self._ck_dict.update({self.agent_name : [ck, curr_time]})

            ck_list = []
            for key in self._ck_dict.keys():
                #rospy.logwarn("%s ck timestamp: %f", key, self._ck_dict[key][1])
                if (curr_time - self._ck_dict[key][1] <= self.dropout_protection_time):
                    ck_list.append(self._ck_dict[key][0])
                else:
                    rospy.logwarn("%s 's last ck value was sent over %f seconds ago and will not be used in the reconstruction", key, self.dropout_protection_time)
            ck_mean = np.mean(ck_list, axis=0)
            #print('sharing and make sure first ck is 0 ', ck[0])
        else:
            ck_mean = ck

        # Difference between the current trajectory and the target distribution.
        fourier_diff = self.lamk * (ck_mean - self.target_dist.phik)
        fourier_diff = fourier_diff.reshape(-1,1)

        ergodic_metric1 = np.sum(self.lamk * np.square(ck_mean - self.target_dist.phik * 0.1))
        #ergodic_metric2 = np.sum(self.lamk * fourier_diff)
        #ergodic_metric3 = np.sum(self.lamk * np.square(fourier_diff))

        ## set max ergodic metric
        if (ergodic_metric1 > self.max_ergodic_metric):
           self.max_ergodic_metric = ergodic_metric1
        norm_ergodic_metric1 = ergodic_metric1/self.max_ergodic_metric

        # backwards pass
        rho = np.zeros(self.model.observation_space.shape[0])
        for t in reversed(range(self.horizon)):
            edx = np.zeros(self.model.observation_space.shape[0])
            edx[self.model.explr_idx] = np.sum(dfk[t] * fourier_diff, 0)

            ## logging edx (which is the ergodic metric?)
            if (self.loop_counter % 300 == 0):
                t_temp = rospy.get_time() - self.start_time
                if (self.agent_name == "rover_0"):
                    with open("{}/{}_{}_{}_{}_{}_{}_ergodicity_timestamps.log".format(self.plotting_dir, cyear, cmonth, cday, chour, cmin, self.agent_name), "a") as f:
                        f.write("timestamp: {}".format(t_temp))
                        f.write('\n')
                        f.write("ergodic_metric1: {}".format(ergodic_metric1))
                        f.write('\n')
                        f.write("norm_ergodic_metric1: {}".format(norm_ergodic_metric1))
                        f.write('\n')
                    self.erg_ts.append(t_temp)
                    self.erg_metric_vals.append(ergodic_metric1)
                    self.norm_erg_metric_vals.append(norm_ergodic_metric1)
                    self.ax.plot(self.erg_ts, self.norm_erg_metric_vals, color="m", marker=".")
                    #self.ax[0].plot(self.erg_ts, self.erg_metric_vals, color="m", marker=".")
                    #self.ax[1].plot(self.erg_ts, self.norm_erg_metric_vals, color="m", marker=".")
                    try:
                        self.fig.canvas.draw()
                    except:
                        self.init_tablet_fig()
                        self.ax.plot(self.erg_ts, self.norm_erg_metric_vals, color="m", marker=".")
                        #self.ax[0].plot(self.erg_ts, self.erg_metric_vals, color="m", marker=".")
                        #self.ax[1].plot(self.erg_ts, self.norm_erg_metric_vals, color="m", marker=".")
                        self.fig.canvas.draw()
                    renderer = self.fig.canvas.renderer
                    self.ax.draw(renderer)
                    #self.ax[0].draw(renderer)
                    #self.ax[1].draw(renderer)
                    plt.pause(0.0001)
                    ## save figure
                    plt.savefig("{}/{}_ergodic_metric_timestamp_{}.pdf".format(self.plotting_dir, self.agent_name, t_temp))
            self.loop_counter = self.loop_counter + 1

            bdx = np.zeros(self.model.observation_space.shape[0])
            bdx[self.model.explr_idx] = dbar[t]
            rho = rho - self.model.dt * (- edx - bdx - np.dot(fdx[t].T, rho))

            self.u_seq[t] = -np.dot(np.dot(self.Rinv, fdu[t].T), rho)
            if (np.abs(self.u_seq[t]) > 1.0).any():
                self.u_seq[t] /= np.linalg.norm(self.u_seq[t])

        self.replay_buffer.push(state[self.model.explr_idx].copy())
        return self.u_seq[0].copy()
