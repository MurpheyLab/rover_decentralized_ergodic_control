#!/usr/bin/env python
import os
import rospy
import numpy as np

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, Float32MultiArray, String

from utils import convert_phi2phik
from rover_decentralized_ergodic_control.srv import ClearMapTablet, ClearMapStomp
from rover_decentralized_ergodic_control.msg import tablet

'''Normalizing the coordinates with respect to the coordinate
bounding box of the area we are deploying the rovers in.
'''


class TargetDist(object):
    '''
    Defines the target distribution that the rovers will explore.
    The distribution is initialized to uniform.

    To edit the target distribution, change the 'means' and 'vars' variables
    to update the means and variance of Gaussian distributions centered at their
    respective means.
    '''
    def __init__(self, basis, num_pts=50):

        # get params from server
        self.width = rospy.get_param("/control_node/dims")
        self.pts = rospy.get_param("/control_node/pts")
        self.agent_id = rospy.get_param("~agent_id")

        self.basis = basis

        # # Square Grid (FX3 code, HumanSwarmCollab simulation tests)
        # grid = np.meshgrid(*[np.linspace(0, 1, num_pts) for _ in range(2)])
        # self.grid = np.c_[grid[0].ravel(), grid[1].ravel()]

        # Non-Square Grids (FX5 code)
        grid = np.meshgrid(np.arange(0,self.width[0],1.0/self.pts), np.arange(0,self.width[1],1.0/self.pts)) # 225m x 450m, Camp Shelby Map
        self.grid = np.c_[grid[0].ravel(), grid[1].ravel()] # dim = (num_pts**2,2) n=2
        self.num_pts = grid[0].shape

        # Initialize with Uniform Exploration
        self.grid_vals = self.__call__(self.grid) # aka phi
        self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
        self.has_update = False

        # For editing the target distribution.
        self.hvt_means = []
        self.ied_means = []
        self.hvt_vars = []
        self.ied_vars = []
        self.has_hvts = False
        self.has_ieds = False

        # subscribers for obtaining the positions of user-specified HVT's and IED's
        hvt_subscriber = rospy.Subscriber("HVT_pose", PoseArray, self.hvt_pose_callback)
        ied_subscriber = rospy.Subscriber("IED_pose", PoseArray, self.ied_pose_callback)
        tablet_hvt_subscriber = rospy.Subscriber("tablet_HVT_pose", PoseArray, self.tablet_hvt_pose_callback)
        tablet_ied_subscriber = rospy.Subscriber("tablet_IED_pose", PoseArray, self.tablet_ied_pose_callback)

        # subscriber to the target distribution being published by the tablet/Tanvas
        # NOTE: we may need to use the custom msg type defined in tablet.msg (in the "tablet" branch of HumanSwarmCollab)
        rospy.Subscriber("tablet_comm", tablet, self.tablet_callback)
        rospy.Subscriber("tanvas_status", Bool, self.tanvas_status_callback)

        # Change to True if we're using the tablet to input HVT/IED locations
        # NOTE: this should be set True/False all the way from the service call from STOMP in ergodic_planner.py
        self.tablet = False

        # Start map clearing services
        self.server_clear_map_tablet()
        self.server_clear_map_stomp()

        # publish the target distribution to our plotting file
        self.target_pub = rospy.Publisher("target_distribution", Float32MultiArray, queue_size=10, latch=True)
        self.tablet_target_pub = rospy.Publisher("tablet_target_distribution", Float32MultiArray, queue_size=10, latch=True)

        # if this is rover_0, take the target distribution received from the tablet and publish it to other agents on /INPUT_TOPIC_3
        #self.other_agent_tablet_pub = rospy.Publisher("INPUT_TOPIC_3", String, queue_size=10, latch=True)

        # Agents that aren't rover_0 will receive the target (from the tablet) on this topic (published by rover_0)
        #rospy.Subscriber("OUTPUT_TOPIC_3", String, self.other_agent_tablet_callback)

    @property
    def phik(self):
        return self._phik

    @phik.setter
    def phik(self, phik):
        assert len(phik) == self.basis.tot_num_basis, 'phik not right dim'
        self._phik = self.phik.copy()

    def get_grid_spec(self):
        xy = []
        for g in self.grid.T:
            xy.append(
                np.reshape(g, newshape=(self.num_pts[0], self.num_pts[1]))
            )
        return xy, self.grid_vals.reshape(self.num_pts[0], self.num_pts[1])

    def server_clear_map_stomp(self):
        s = rospy.Service('clear_map_stomp', ClearMapStomp, self.handle_clear_map_stomp)
        rospy.loginfo_once('Service for clearing map (from STOMP) initialized')

    def handle_clear_map_stomp(self, req):
        # clear the map (STOMP)
        self.grid_vals = self.__call__(self.grid)
        self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
        self.has_update = True

        self.hvt_means = []
        self.ied_means = []
        self.hvt_vars = []
        self.ied_vars = []
        self.has_hvts = False
        self.has_ieds = False

        # publish the target_dist after claering map
        target = Float32MultiArray()
        target.data = self.grid_vals.copy()
        self.target_pub.publish(target)

        response = "Service call to clear STOMP HVT/IED positions received"
        rospy.logwarn("map cleared")
        return response

    def server_clear_map_tablet(self):
        s = rospy.Service('clear_map_tablet', ClearMapTablet, self.handle_clear_map_tablet)
        rospy.loginfo_once('Service for clearing map (from tablet) initialized')

    def handle_clear_map_tablet(self, req):
        # clear the map (tablet)
        self.grid_vals = self.__call__(self.grid)
        self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
        self.has_update = True

        self.hvt_means = []
        self.ied_means = []
        self.hvt_vars = []
        self.ied_vars = []
        self.has_hvts = False
        self.has_ieds = False

        # publish the target_dist after clearing map
        target = Float32MultiArray()
        target.data = self.grid_vals.copy()
        self.target_pub.publish(target)

        response = "Service call to clear tablet/tanvas map received."
        rospy.logwarn("map cleared")
        return response

    def __call__(self, x): # init_uniform_grid(self,x) in rviz code
        assert len(x.shape) > 1, 'Input needs to be a of size N x n'
        assert x.shape[1] == 2, 'Does not have right exploration dim'

        val = np.ones(x.shape[0])
        val /= np.sum(val)

        return val

    def hvt_pose_callback(self, msg):
        received_hvts = msg.poses
        for hvt in received_hvts:
            rospy.logwarn('Added a new HVT position to the distribution')

            self.hvt_means.append(np.array([hvt.position.x, hvt.position.y]))
            self.hvt_vars.append(np.array([0.01, 0.01]))

            # print the hvt_means array to confirm we still have the earlier hvt's
            rospy.logwarn('New HVT values: ')
            rospy.logwarn(self.hvt_means)

            self.has_hvts = True
            hvt_val = np.zeros(self.grid.shape[0])
            hvt_scale = np.ones(self.grid.shape[0])
            for m, v in zip(self.hvt_means, self.hvt_vars):
                innerds = np.sum((self.grid-m)**2 / v, 1)
                hvt_val += np.exp(-innerds/2.0)
            hvt_val /= np.sum(hvt_val)
            hvt_scale = hvt_val #not used

            ied_scale = np.ones(self.grid.shape[0])
            ied_val = np.zeros(self.grid.shape[0])
            if self.has_ieds:
                for m, v in zip(self.ied_means, self.ied_vars):
                    innerds = np.sum((self.grid-m)**2 / v, 1)
                    ied_val += np.exp(-innerds/2.0)
                # Invert IED distribution
                ied_val -= np.max(ied_val)
                ied_val = np.abs(ied_val) #1e-5
                ied_val /= np.sum(ied_val)

                ied_scale = ied_val

            # # This section was from the HumanSwarmCollab code but I don't think we need the tablet section here anymore
            # if self.tablet:
            #     val = (self.tablet_dist + hvt_val + ied_val) * ied_scale
            # else:
            #     val = (hvt_val + ied_val) * ied_scale
            val = (hvt_val + ied_val) * ied_scale


            # normalizes the distribution:
            val /= np.sum(val)
            self.grid_vals = val
            self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
            rospy.logwarn("showing max and min values of self.grid_vals in the form: [hvt_val min, hvt_val max, ied_val min, ied_val max, val min, val max]")
            rospy.logwarn([np.min(hvt_val),np.max(hvt_val),np.min(ied_val),np.max(ied_val),np.min(val),np.max(val)])

        # publish the target_dist after adding the ieds
        target = Float32MultiArray()
        target.data = self.grid_vals.copy()
        self.target_pub.publish(target)

        # denote that the target distribution has been updated
        self.has_update = True

    def ied_pose_callback(self, msg):
        received_ieds = msg.poses
        for ied in received_ieds:
            rospy.logwarn('Added a new IED position to the distribution')

            self.ied_means.append(np.array([ied.position.x, ied.position.y]))
            #self.ied_vars.append(np.array([0.01, 0.01]))
            self.ied_vars.append(np.array([0.1, 0.1])**2)

            # print new IED values
            rospy.logwarn('New IED values: ')
            rospy.logwarn(self.ied_means)
            self.has_ieds = True

            hvt_val = np.zeros(self.grid.shape[0])
            hvt_scale = np.ones(self.grid.shape[0])
            if self.has_hvts:
                for m, v in zip(self.hvt_means, self.hvt_vars):
                    innerds = np.sum((self.grid-m)**2 / v, 1)
                    hvt_val += np.exp(-innerds/2.0)
                hvt_val /= np.sum(hvt_val)
                hvt_scale = hvt_val

            ied_val = np.zeros(self.grid.shape[0])
            ied_scale = np.ones(self.grid.shape[0])
            for m, v in zip(self.ied_means, self.ied_vars):
                innerds = np.sum((self.grid-m)**2 / v, 1)
                ied_val += np.exp(-innerds/2.0)

            # Invert IED distribution
            ied_val -= np.max(ied_val)
            ied_val = np.abs(ied_val) #1e-5
            ied_val /= np.sum(ied_val)

            ied_scale = ied_val

            # # This section was from the HumanSwarmCollab code but I don't think we need the tablet section here anymore
            # if self.tablet:
            #     val = (self.tablet_dist + hvt_val + ied_val) * ied_scale
            # else:
            #     val = (hvt_val + ied_val) * ied_scale
            val = (hvt_val + ied_val) * ied_scale

            # normalizes the distribution
            val /= np.sum(val)
            self.grid_vals = val
            self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
            rospy.logwarn("showing max and min values of self.grid_vals in the form: [hvt_val min, hvt_val max, ied_val min, ied_val max, val min, val max]")
            rospy.logwarn([np.min(hvt_val),np.max(hvt_val),np.min(ied_val),np.max(ied_val),np.min(val),np.max(val)])

        # publish the target_dist after adding the ieds
        target = Float32MultiArray()
        target.data = self.grid_vals.copy()
        self.target_pub.publish(target)

        # denote that the target distribution has been updated
        self.has_update = True

    def tablet_hvt_pose_callback(self, msg):
        received_hvts = msg.poses
        for hvt in received_hvts:
            rospy.logwarn('Added a new HVT position to the distribution')

            self.hvt_means.append(np.array([hvt.position.x, hvt.position.y]))
            self.hvt_vars.append(np.array([0.01, 0.01]))

            # print the hvt_means array to confirm we still have the earlier hvt's
            rospy.logwarn('New HVT values: ')
            rospy.logwarn(self.hvt_means)

            self.has_hvts = True
            hvt_val = np.zeros(self.grid.shape[0])
            hvt_scale = np.ones(self.grid.shape[0])
            for m, v in zip(self.hvt_means, self.hvt_vars):
                innerds = np.sum((self.grid-m)**2 / v, 1)
                hvt_val += np.exp(-innerds/2.0)
            hvt_val /= np.sum(hvt_val)
            hvt_scale = hvt_val #not used

            ied_scale = np.ones(self.grid.shape[0])
            ied_val = np.zeros(self.grid.shape[0])
            if self.has_ieds:
                for m, v in zip(self.ied_means, self.ied_vars):
                    innerds = np.sum((self.grid-m)**2 / v, 1)
                    ied_val += np.exp(-innerds/2.0)
                # Invert IED distribution
                ied_val -= np.max(ied_val)
                ied_val = np.abs(ied_val) #1e-5
                ied_val /= np.sum(ied_val)

                ied_scale = ied_val

            # # This section was from the HumanSwarmCollab code but I don't think we need the tablet section here anymore
            # if self.tablet:
            #     val = (self.tablet_dist + hvt_val + ied_val) * ied_scale
            # else:
            #     val = (hvt_val + ied_val) * ied_scale
            val = (hvt_val + ied_val) * ied_scales


            # normalizes the distribution:
            val /= np.sum(val)
            self.grid_vals = val
            self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
            rospy.logwarn("showing max and min values of self.grid_vals in the form: [hvt_val min, hvt_val max, ied_val min, ied_val max, val min, val max]")
            rospy.logwarn([np.min(hvt_val),np.max(hvt_val),np.min(ied_val),np.max(ied_val),np.min(val),np.max(val)])

        # publish the target_dist after adding the hvts
        target = Float32MultiArray()
        target.data = self.grid_vals.copy()
        self.tablet_target_pub.publish(target)

        # denote that the target distribution has been updated
        self.has_update = True

    def tablet_ied_pose_callback(self, msg):
        received_ieds = msg.poses
        for ied in received_ieds:
            rospy.logwarn('Added a new IED position to the distribution')

            self.ied_means.append(np.array([ied.position.x, ied.position.y]))
            #self.ied_vars.append(np.array([0.01, 0.01]))
            self.ied_vars.append(np.array([0.1, 0.1])**2)

            # print new IED values
            rospy.logwarn('New IED values: ')
            rospy.logwarn(self.ied_means)
            self.has_ieds = True

            hvt_val = np.zeros(self.grid.shape[0])
            hvt_scale = np.ones(self.grid.shape[0])
            if self.has_hvts:
                for m, v in zip(self.hvt_means, self.hvt_vars):
                    innerds = np.sum((self.grid-m)**2 / v, 1)
                    hvt_val += np.exp(-innerds/2.0)
                hvt_val /= np.sum(hvt_val)
                hvt_scale = hvt_val

            ied_val = np.zeros(self.grid.shape[0])
            ied_scale = np.ones(self.grid.shape[0])
            for m, v in zip(self.ied_means, self.ied_vars):
                innerds = np.sum((self.grid-m)**2 / v, 1)
                ied_val += np.exp(-innerds/2.0)
            ied_scale = ied_val

            # Invert IED distribution
            ied_val -= np.max(ied_val)
            ied_val = np.abs(ied_val) #1e-5
            ied_val /= np.sum(ied_val)

            # # This section was from the HumanSwarmCollab code but I don't think we need the tablet section here anymore
            # if self.tablet:
            #     val = (self.tablet_dist + hvt_val + ied_val) * ied_scale
            # else:
            #     val = (hvt_val + ied_val) * ied_scale
            val = (hvt_val + ied_val) * ied_scale

            # normalizes the distribution
            val /= np.sum(val)
            self.grid_vals = val
            self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
            rospy.logwarn("showing max and min values of self.grid_vals in the form: [hvt_val min, hvt_val max, ied_val min, ied_val max, val min, val max]")
            rospy.logwarn([np.min(hvt_val),np.max(hvt_val),np.min(ied_val),np.max(ied_val),np.min(val),np.max(val)])

        # publish the target_dist after adding the ieds
        target = Float32MultiArray()
        target.data = self.grid_vals.copy()
        self.tablet_target_pub.publish(target)

        # denote that the target distribution has been updated
        self.has_update = True

    def tanvas_status_callback(self, msg):
        self.tablet = msg.data

    def tablet_callback(self, msg):
        '''Receives target distributions drawn in Tom's tablet interface; publishes them so they can be viewed by the user via plotting.py.'''
        rospy.logwarn("received a new target drawn on the tablet")
        if (self.tablet):                                                   #only use targets coming from the tablet if this agent is set to use the tablet
            rospy.logwarn("this agent is set to use the tablet")
            self.grid_vals = np.asarray(msg.data)
            rospy.logwarn("showing max and min values of self.grid_vals in the form: [min, max]")
            rospy.logwarn([np.amin(self.grid_vals.copy()),np.amax(self.grid_vals.copy())])
            self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
            target = Float32MultiArray()
            target.data = self.grid_vals.copy()
            self.tablet_target_pub.publish(target)                          # publish target so it can be plotted by plotting.py

            # # since BBN's comms aren't working, we are publishing the target from here to /INPUT_TOPIC_3 to send it to the other swarm members (same method used for ck sharing)
            # target_list = self.grid_vals.copy().tolist()
            # target_string_msg = String()
            # target_string_msg.data = ' '.join([str(elem) for elem in target_list])
            # rospy.logwarn(target_string_msg.data)
            # self.other_agent_tablet_pub.publish(target_string_msg)

            # denote that the target distribution has been updated
            self.has_update = True
            
        # else:
        #     rospy.logwarn("target received from tablet in tablet_callback, but this agent's self.tablet value = False")
        #     if (self.agent_id == "rover_0"):
        #         rospy.logwarn("still broadcasting the tablet target from rover_0 -> the other agents")
        #         target = np.asarray(msg.data)
        #         target_list = target.tolist()
        #         target_string_msg = String()
        #         target_string_msg.data = ' '.join([str(elem) for elem in target_list])
        #         self.other_agent_tablet_pub.publish(target_string_msg)
                
                

    # def other_agent_tablet_callback(self, msg):
    #     '''Receives the tablet's target distribution from rover_0.'''
    #     if (self.tablet and not (self.agent_id == "rover_0")):
    #         rospy.logwarn("received a target (originally from the tablet) broadcast by rover_0")
    #         msg_str = msg.data
    #         converted_data = np.array(list(map(float, msg_str.split(' '))))
    #         self.grid_vals = converted_data
    #         self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
            
    #         target = Float32MultiArray()
    #         target.data = self.grid_vals.copy()
    #         self.tablet_target_pub.publish(target)                            # publish target so it can be plotted by plotting.py

    #         # denote that the target distribution has been updated
    #         self.has_update = True
        
