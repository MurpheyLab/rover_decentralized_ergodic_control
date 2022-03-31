#!/usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import rospy
import numpy as np
import time

from target_dist import TargetDist
from basis import Basis
from utils import convert_phi2phik, convert_phik2phi, convert_ck2dist
from single_integrator import SingleIntegrator

from std_msgs.msg import String, Bool, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose

from rover_decentralized_ergodic_control.srv import ClearMapTablet, ClearMapStomp

from collections import defaultdict

import os #used to get ROS_MASTER_URI env variable

import datetime
curr_date_time = datetime.datetime.now()
cmonth = curr_date_time.strftime("%b")
cday = curr_date_time.strftime("%d")
chour = curr_date_time.strftime("%H")
cmin = curr_date_time.strftime("%M")
csec = curr_date_time.strftime("%S")
cyear = curr_date_time.strftime("%Y")

'''Normalizing the coordinates with respect to the coordinate
bounding box of the area we are deploying the rovers in.
'''

class Plotting(object):
        def __init__(self):
            # create a rospy node
            rospy.init_node('plotter')

            # set plotting rate
            self.rate = rospy.Rate(10)

            # wait for stomp before building node
            test = rospy.wait_for_message("INPUT_TOPIC_2", String)
            # get params from server
            self.width = rospy.get_param("/control_node/dims")
            self.pts = rospy.get_param("/control_node/pts")

            self._ck_dict = defaultdict(list)
            self._ck_dict2 = defaultdict(list)

            # if a ck value from a particular agent was last received > dropout_protection_time; don't use that agent's ck in the reconstruction
            self.dropout_protection_time = rospy.get_param("/control_node/dropout_protection_time")

            # importing the model we're using
            self.model = SingleIntegrator()
            self.basis = Basis(self.model.explr_space, num_basis=15)

            # rectangular grid
            grid = np.meshgrid(np.arange(0,self.width[0],1.0/self.pts), np.arange(0,self.width[1],1.0/self.pts)) # 225m x 450m, Camp Shelby Map
            self.grid = np.c_[grid[0].ravel(), grid[1].ravel()]
            self.num_pts = grid[0].shape

            # averaged ck's from all agents -- using this to reconstruct the trajectories
            self.ck_mean = None        #STOMP
            self.ck_mean2 = None       #Tanvas/Tablet

            # placeholder until figures are initialized
            self.figure = None
            self.figure2 = None

            # wait for ck values to start being shared amongst the agents
            self.received_cks = False   #STOMP
            self.received_cks2 = False  #Tanvas/tablet
            self.last_ck_time = None    #STOMP
            self.last_ck_time2 = None   #Tanvas/tablet

            # wait for target distribution to be published
            self.received_target_distribution = False
            self.received_tablet_target_distribution = False

            # the current target distributions for both STOMP and tanvas/tablet
            self.grid_vals = None
            self.grid_vals2 = None

            # Changes to True if we're using the Tanvas
            self.tanvas = False

            # Start map clearing services
            self.server_clear_map_tablet()
            self.server_clear_map_stomp()

            ## keeping track of what target specification we are on (this starts at -2, then gets incremented every time a new target distribution
            ## is received from the user; starting at -2 b/c launching the tactic from stomp for some reason triggers the callback twice
            self.specification_number = -2

            ## use os to get ROS_MASTER_URI to set the agent (rover) name
            ros_master_uri_string = os.environ.get("ROS_MASTER_URI")
            #print("ros_master_uri_string is: {}".format(ros_master_uri_string))

            if (ros_master_uri_string):
                if (ros_master_uri_string[-5:] == "11312"):
                    self.agent_name = "rover_0"
                elif (ros_master_uri_string[-5:] == "11313"):
                    self.agent_name = "rover_1"
                elif (ros_master_uri_string[-5:] == "11314"):
                    self.agent_name = "rover_2"
                elif (ros_master_uri_string[-5:] == "11315"):
                    self.agent_name = "rover_3"
                elif (ros_master_uri_string[-5:] == "11316"):
                    self.agent_name = "rover_4"
                elif (ros_master_uri_string[-5:] == "11317"):
                    self.agent_name = "rover_5"
                else:
                    self.agent_name=None
            print("self.agent_name is: {}".format(self.agent_name))

            ## create the plotting directory
            real_path=os.path.realpath(__file__)
            base_path = os.path.dirname(real_path)
            self.plotting_dir = base_path+"/data/{}_{}_{}_{}_{}_{}".format(cyear, cmonth, cday, chour, cmin, self.agent_name)
            print("self.plotting_dir is: {}".format(self.plotting_dir))
            if not(os.path.exists(self.plotting_dir)):
                try:
                    os.makedirs(self.plotting_dir)
                except:
                    print("couldn't make plotting directory")

            self.loop_counter = 0
            # subscribing to ck sharing topic for other agents
            rospy.Subscriber('OUTPUT_TOPIC_1', String, self.ck_list_callback)  #topic for sharing ck's from agents running a target_dist from STOMP
            rospy.Subscriber('OUTPUT_TOPIC_2', String, self.ck_list_callback2) #topic for sharing ck's from agents running a target_dist from the tablet
            # subscribing to ck sharing topic for self
            rospy.Subscriber('INPUT_TOPIC_1', String, self.ck_list_callback)  #topic for sharing ck's from agents running a target_dist from STOMP
            rospy.Subscriber('INPUT_TOPIC_2', String, self.ck_list_callback2)  #topic for sharing ck's from agents running a target_dist from STOMP

            # subscribe to spatial distribution (phi) sent from target_dist.py
            rospy.Subscriber("target_distribution", Float32MultiArray, self.target_dist_callback)
            rospy.Subscriber("tablet_target_distribution", Float32MultiArray, self.tablet_target_dist_callback)

            # Status variable -- are we using the tablet/tanvas?
            rospy.Subscriber("tanvas_status", Bool, self.tanvas_status_callback)

        def init_stomp_fig(self):
            # STOMP plots
            self.figure = plt.figure(figsize=(5,5))
            self.ax = self.figure.add_subplot(2,1,1)
            self.ax.set_title('Time Averaged Agent Trajectories (STOMP)')
            self.ax.set_xlabel('x position [m]')
            self.ax.set_ylabel('y position [m]')

            self.ax2 = self.figure.add_subplot(2,1,2)
            self.ax2.set_title('Target Distribution (STOMP)')
            self.ax2.set_xlabel('x position [m]')
            self.ax2.set_ylabel('y position [m]')

            # Colorbar for STOMP plots
            self.sm = plt.cm.ScalarMappable(Normalize(0,1), cmap='RdBu')
            self.sm.set_array(range(0,1))
            self.cbar_ax = self.figure.colorbar(self.sm, ax=self.ax, ticks=[0,1])
            self.cbar_ax.set_ticklabels(['Low', 'High'])
            self.cbar_ax2 = self.figure.colorbar(self.sm, ax=self.ax2, ticks=[0,1])
            self.cbar_ax2.set_ticklabels(['Low', 'High'])
            self.figure.tight_layout()

        def init_tablet_fig(self):
            #Tanvas/tablet plots
            self.figure2 = plt.figure(figsize=(5,5))
            self.ax3 = self.figure2.add_subplot(2,1,1)
            self.ax3.set_title('Time Averaged Agent Trajectories (Tablet)')
            self.ax3.set_xlabel('x position [m]')
            self.ax3.set_ylabel('y position [m]')

            self.ax4 = self.figure2.add_subplot(2,1,2)
            self.ax4.set_title('Target Distribution (Tablet)')
            self.ax4.set_xlabel('x position [m]')
            self.ax4.set_ylabel('y position [m]')

            # Colorbar for tablet plots
            self.sm2 = plt.cm.ScalarMappable(Normalize(0,1), cmap='RdBu')
            self.sm2.set_array(range(0,1))
            self.cbar_ax3 = self.figure2.colorbar(self.sm2, ax=self.ax3, ticks=[0,1])
            self.cbar_ax3.set_ticklabels(['Low', 'High'])
            self.cbar_ax4 = self.figure2.colorbar(self.sm2, ax=self.ax4, ticks=[0,1])
            self.cbar_ax4.set_ticklabels(['Low', 'High'])

            #plt.tight_layout()
            self.figure2.tight_layout()


        def target_dist_callback(self, msg):
            rospy.logwarn("Received target_dist")
            self.grid_vals = np.asarray(msg.data)
            self.received_target_distribution = True

        def tablet_target_dist_callback(self, msg):
            if self.received_tablet_target_distribution:
                try:
                    plt.savefig("{}/{}_specification_{}_globaltimestamp_{}.pdf".format(self.plotting_dir, self.agent_name, self.specification_number, time.time()),dpi=25)
                    print('saved figure in tablet target dist callback')
                except:
                    print('unable to save figure')

            rospy.logwarn("Received tablet target_dist")
            self.grid_vals2 = np.asarray(msg.data)
            self.received_tablet_target_distribution = True

            ## increment the specification number every time a new target is received from the touchscreen
            self.specification_number = self.specification_number + 1


        def init_uniform(self, x):
            assert len(x.shape) > 1, 'Input needs to be a of size N x n'
            assert x.shape[1] == 2, 'Does not have right exploration dim'

            val = np.ones(x.shape[0])
            val /= np.sum(val)
            return val

        def server_clear_map_stomp(self):
            s = rospy.Service('clear_map_stomp', ClearMapStomp, self.handle_clear_map_stomp)
            rospy.loginfo_once('Plotter Service for clearing map (from STOMP) initialized')

        def handle_clear_map_stomp(self, req):
            # clear the map (STOMP)
            self.grid_vals = self.init_uniform(self.grid)

            response = "Plotter Service call to clear STOMP HVT/IED positions received"
            return response

        def server_clear_map_tablet(self):
            s = rospy.Service('clear_map_tablet', ClearMapTablet, self.handle_clear_map_tablet)
            rospy.loginfo_once('Plotter Service for clearing map (from tablet) initialized')

        def handle_clear_map_tablet(self, req):
            # clear the map (tablet)
            self.grid_vals2 = self.init_uniform(self.grid)

            response = "Plotter Service call to clear tablet/tanvas map received."
            return response

        def tanvas_status_callback(self, msg):
            self.tanvas = msg.data

        def ck_list_callback(self, msg):
            ck_time_received = rospy.get_time()

            msg_str = msg.data
            msg_str_to_list = list(msg_str.split(" "))

            if self._ck_dict.has_key(msg_str_to_list[0]):
                self._ck_dict[msg_str_to_list[0]][0] = np.array(msg_str_to_list[1:], dtype=np.float32)
                self._ck_dict[msg_str_to_list[0]][1] = ck_time_received
            else:
                self._ck_dict.update({msg_str_to_list[0] : [np.array(msg_str_to_list[1:], dtype=np.float32), ck_time_received]})

            # update boolean
            self.received_cks = True
            self.last_ck_time = time.time()

        def ck_list_callback2(self, msg):
            ck_time_received = rospy.get_time()

            msg_str = msg.data
            msg_str_to_list = list(msg_str.split(" "))

            if self._ck_dict2.has_key(msg_str_to_list[0]):
                self._ck_dict2[msg_str_to_list[0]][0] = np.array(msg_str_to_list[1:], dtype=np.float32)
                self._ck_dict2[msg_str_to_list[0]][1] = ck_time_received
            else:
                self._ck_dict2.update({msg_str_to_list[0] : [np.array(msg_str_to_list[1:], dtype=np.float32), ck_time_received]})

            # update boolean
            self.received_cks2 = True
            self.last_ck_time2 = time.time()

        def average_cks(self):
            curr_time = rospy.get_time()

            print('calculating a new ck_mean value')
            ck_list = []
            for key in self._ck_dict.keys():
                if (curr_time - self._ck_dict[key][1] <= self.dropout_protection_time):
                    ck_list.append(self._ck_dict[key][0])
                else:
                    rospy.logwarn("%s 's last ck value was sent over %f seconds ago and will not be used in the reconstruction", key, self.dropout_protection_time)
            ck_mean = np.mean(ck_list, axis=0)
            return ck_mean

        def average_cks2(self):
            curr_time = rospy.get_time()

            print('calculating a new ck_mean value (for Tanvas/Tablet target distribution)')
            ck_list2 = []
            for key in self._ck_dict2.keys():
                if (curr_time - self._ck_dict2[key][1] <= self.dropout_protection_time):
                    ck_list2.append(self._ck_dict2[key][0])
                else:
                    rospy.logwarn("%s 's last ck value was sent over %f seconds ago and will not be used in the reconstruction", key, self.dropout_protection_time)
            ck_mean2 = np.mean(ck_list2, axis=0)
            return ck_mean2

        def plot(self):
            if self.ck_mean is not None:
                dist_mean = convert_ck2dist(self.basis, self.ck_mean, grid=self.grid)
                dist_mean_reshaped = np.reshape(dist_mean, (self.num_pts[0],self.num_pts[1]))

                print('generating new plot (for STOMP)...')
                self.ax.imshow(dist_mean_reshaped, interpolation='none', cmap='RdBu', origin='lower')

                try:
                    self.figure.canvas.draw()
                    renderer = self.figure.canvas.renderer
                    self.ax.draw(renderer)
                    plt.pause(0.0001)
                except:
                    self.figure = None
                    return

        def plot_target_dist(self):
            if self.grid_vals is not None:
                grid_vals_reshaped = np.reshape(self.grid_vals, (self.num_pts[0],self.num_pts[1]))

                print('generating target_dist plot (for STOMP)...')
                self.ax2.imshow(grid_vals_reshaped, interpolation='none', cmap='RdBu', origin='lower')

                try:
                    self.figure.canvas.draw()
                    renderer = self.figure.canvas.renderer
                    self.ax2.draw(renderer)
                    plt.pause(0.0001)
                except:
                    self.figure = None
                    return

        def plot_tanvas(self):
            if self.ck_mean2 is not None:
                dist_mean2 = convert_ck2dist(self.basis, self.ck_mean2, grid=self.grid)
                dist_mean2_reshaped = np.reshape(dist_mean2, (self.num_pts[0],self.num_pts[1]))

                print('generating new plot (for Tanvas/Tablet)...')
                self.ax3.imshow(dist_mean2_reshaped, interpolation='none', cmap='RdBu', origin='lower')

                try:
                    self.figure2.canvas.draw()
                    renderer = self.figure2.canvas.renderer
                    self.ax3.draw(renderer)
                    plt.pause(0.0001)

                    # save figure
                    if (self.loop_counter % 20 == 0):
                        plt.savefig("{}/{}_specification_{}_globaltimestamp_{}.pdf".format(self.plotting_dir, self.agent_name, self.specification_number, time.time()),dpi=25)
                except:
                    self.figure2 = None
                    return

        def plot_tanvas_target_dist(self):
            if self.grid_vals2 is not None:
                grid_vals_reshaped2 = np.reshape(self.grid_vals2, (self.num_pts[0],self.num_pts[1]))

                print('generating target_dist plot (for Tanvas/Tablet)...')
                self.ax4.imshow(grid_vals_reshaped2, interpolation='none', cmap='RdBu', origin='lower')

                try:
                    self.figure2.canvas.draw()
                    renderer = self.figure2.canvas.renderer
                    self.ax4.draw(renderer)
                    plt.pause(0.0001)
                except:
                    self.figure2 = None
                    return



if __name__ == '__main__':
    plotter_node = Plotting()

    try:
        while not rospy.is_shutdown():
            # STOMP target and reconstruction plots
            if (plotter_node.received_target_distribution and plotter_node.received_cks):
                if (not plotter_node.tanvas):
                    plotter_node.ck_mean = plotter_node.average_cks()
                    if plotter_node.figure is None: plotter_node.init_stomp_fig()
                    plotter_node.plot()                               # plot the reconstructed trajectories of all the agents (run from STOMP)
                    if plotter_node.figure is None: plotter_node.init_stomp_fig()
                    plotter_node.plot_target_dist()                   # plot the target_dist (run from STOMP)

                else:
                    rospy.logwarn("This agent is not currently set to use STOMP")
                    if (plotter_node.figure):
                        plt.close(plotter_node.figure)

                if (time.time() - plotter_node.last_ck_time ) > 20: # seconds
                    plotter_node.received_cks = False
                    rospy.logwarn("No messages received from agents in last 20 seconds.")

            # Tablet/Tanvas target and reconstruction plots
            if (plotter_node.received_tablet_target_distribution and plotter_node.received_cks2):
                if plotter_node.figure2 is None: plotter_node.init_tablet_fig()

                if (plotter_node.tanvas):
                    plotter_node.ck_mean2 = plotter_node.average_cks2()
                    if plotter_node.figure2 is None: plotter_node.init_tablet_fig()
                    plotter_node.plot_tanvas()                       # plot the reconstructed trajectories of all the agents (run from tablet/tanvas)
                    if plotter_node.figure2 is None: plotter_node.init_tablet_fig()
                    plotter_node.plot_tanvas_target_dist()           # plot the target_dist (run from tablet/tanvas)

                else:
                    rospy.logwarn("This agent is not currently set to use the tablet/tanvas")
                    if (plotter_node.figure2):
                        plt.close(plotter_node.figure2)

                if (time.time() - plotter_node.last_ck_time2 ) > 20: # seconds
                    plotter_node.received_cks = False
                    rospy.logwarn("No messages received from agents in last 20 seconds.")
            plotter_node.loop_counter = plotter_node.loop_counter + 1
            plotter_node.rate.sleep()

    except rospy.ROSInterruptException as e:
        print('clean break')

    plotter_node.loop_counter=0
    plotter_node.plot_tanvas()
