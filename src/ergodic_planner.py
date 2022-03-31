#!/usr/bin/env python

import rospy
import actionlib

import numpy as np
import math

from position_controller import PositionController
from utilities import Utilities
from global_pose import GlobalPose

from global_position_controller.srv import GoalPosition
# from ccast_services.srv import GetHardwarePlatformUid

from std_msgs.msg import Float64, String, Bool, Int32, Empty
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
from move_base_msgs.msg import MoveBaseAction

from rt_erg_lib import Agent
from rover_decentralized_ergodic_control.srv import RosErgodicControl

# gps_common is required for using the actual rover hardware
from gps_common.msg import GPSFix

import datetime
curr_date_time = datetime.datetime.now()
cmonth = curr_date_time.strftime("%b")
cday = curr_date_time.strftime("%d")
chour = curr_date_time.strftime("%H")
cmin = curr_date_time.strftime("%M")
csec = curr_date_time.strftime("%S")
cyear = curr_date_time.strftime("%Y")

import time
START_TIME = time.time()

import pandas as pd
import os

class ControlNode(Agent):
    def __init__(self):

        rospy.init_node("control_node", anonymous=True)

        ## dataframes for storing gps and localpose data and then later saving it to file
        #self.df_gps = pd.DataFrame()
        self.df_localpose = pd.DataFrame()

        self._controller    = PositionController()
        self._utilities     = Utilities()
        self._pose          = GlobalPose()
        self._target_pose   = GlobalPose()
        self._check_msg     = Float64()

        self._control_status = 'idle' #default state

        self._rate = 1
        self.rate = rospy.Rate(self._rate)

        # get params from server
        NE_corner = rospy.get_param("/control_node/NE_corner")
        SW_corner = rospy.get_param("/control_node/SW_corner")

        # set up bounding coordinates
        self._coordNE = GlobalPose()
        self._coordSW = GlobalPose()
        self._coordNE.latitude  = NE_corner[0]
        self._coordNE.longitude = NE_corner[1]
        self._coordSW.latitude  = SW_corner[0]
        self._coordSW.longitude = SW_corner[1]
        self._delta_lat = self._coordNE.latitude - self._coordSW.latitude
        # self._delta_lat = self._coordSW.latitude - self._coordNE.latitude
        # self._delta_lon = self._coordNE.longitude - self._coordSW.longitude # not used

        # set up subscribers / publishers
        #rospy.Subscriber('gps_fix', GPSFix, self.gps_callback)
        #rospy.Subscriber('manage_controller', String, self.manage_callback)
        self._control_status_publisher = rospy.Publisher('manage_controller', String, queue_size=1)

        self._agent_id_pub = rospy.Publisher('agent_id', String, queue_size = 1)

        self._client_goal = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 1)
        self._pub_check = rospy.Publisher('controller_check', Float64, queue_size = 1)

        ''' # commenting out block because unused
        self.curr_lat = None
        self.curr_lon = None

        rospy.Subscriber('grid_pts', PoseArray, self.get_grid)

        self.pose_publisher = rospy.Publisher('target_pose', PoseStamped, queue_size = 1)
        self.goal_pose = GoalPosition()
        self.goal_pose.target_heading = -1
        '''
        # uncomment this for testing in hardware: GetHardwarePlatformUid Service call can only be used on actual platforms, not in simulation
        # self.agent_id = rospy.ServiceProxy('get_hardware_platform_uid', GetHardwarePlatformUid)().uid

        # uncomment this for testing in simulation: OUTPUT_TOPIC_1 works with this
        self.agent_id = rospy.get_param('~agent_id', default='rover')
        show_in_rviz = rospy.get_param('~show_in_rviz', default=False)

        # uncomment this when we're testing with hardware
        #self.convert_agent_id_to_name()

        # uncomment this when we're testing in simulation
        self.agent_name = self.agent_id

        print('agent name is', self.agent_name)
        agent_id_msg = String()
        agent_id_msg.data = self.agent_id
        self._agent_id_pub.publish(agent_id_msg)

        self._hvt_publisher = rospy.Publisher('HVT_pose', PoseArray, queue_size=10,latch=True)
        self._ied_publisher = rospy.Publisher('IED_pose', PoseArray, queue_size=10,latch=True)
        self.tanvas_status_publisher = rospy.Publisher('tanvas_status', Bool, queue_size=1)

        #tablet hvt/ied publishers for 1/8/2021 demo (showing concept of STOMP + tablet)
        self._tablet_hvt_publisher = rospy.Publisher('tablet_HVT_pose', PoseArray, queue_size=10,latch=True)
        self._tablet_ied_publisher = rospy.Publisher('tablet_IED_pose', PoseArray, queue_size=10,latch=True)

        #Are we using the tanvas?
        self.tanvas = False

        # Publish the # of HVT/IED positions received from STOMP
        #self.stomp_target_number_publisher = rospy.Publisher('stomp_number_of_targets', Int32, queue_size=10)
        #self.stomp_target_number = None

        # Initialize the agent
        Agent.__init__(self, self.agent_name, show_in_rviz)

        # set up subscribers / publishers
        rospy.Subscriber('gps_fix', GPSFix, self.gps_callback)
        rospy.Subscriber('manage_controller', String, self.manage_callback)

        # subscriber for killing the agent if it's in a particular region of the map
        rospy.Subscriber("kill_agent", Empty, self.kill_agent_callback)
        self.received_kill_command = False
        self.kill_latitude = 31.137100
        self.kill_longitude = -89.063600

    def kill_agent_callback(self, msg):
        self.received_kill_command = True
    
    def manage_callback(self, msg):
        self._control_status = msg.data

    def get_control_status(self):
        return self._control_status

    def stop_moving_platform(self):
        # rospy.logwarn('Attempting to cancel move_base goals')
        self._client_goal.cancel_goal()
        self._client_goal.cancel_all_goals()


    # uncomment this when we're testing in real hardware: this is a function for converting the name of each agent into the required format
    # def convert_agent_id_to_name(self):
    #     agent_id = ''
    #     for i in range(len(self.agent_id[4:])):
    #         agent_id += str(ord(self.agent_id[i]))
    #     self.agent_name = int(agent_id)


    def gps_callback(self, msg):
        rospy.logwarn_once("updating global pose using gps")
        self._pose.latitude = msg.latitude
        self._pose.longitude = msg.longitude
        self._pose.heading = msg.dip * math.pi / 180.0

        local_pose = self._coord_to_dist(self._pose)
        self.state = np.array(local_pose) # updates state for ergodic_control

        ## get current time
        curr_time = time.time() - START_TIME

        ## write the local pose data to file
        #with open("{}_{}_{}_{}_{}_{}_{}_localpose_data.txt".format(cyear, cmonth, cday, chour, cmin, csec, self.agent_name), "a") as f:
        # with open("{}_localpose_data.txt".format(self.agent_name), "a") as f:
        #     f.write("timestamp: {}\n".format(curr_time))
        #     f.write("local [x,y] pose is: {}\n".format(local_pose))

        ## write the gps data to file
        #with open("{}_{}_{}_{}_{}_{}_{}_gps_data.txt".format(cyear, cmonth, cday, chour, cmin, csec, self.agent_name), "a") as f:
        # with open("{}_gps_data.txt".format(self.agent_name), "a") as f:
        #     f.write("timestamp: {}\n".format(curr_time))
        #     f.write("lat is: {}\n".format(self._pose.latitude))
        #     f.write("lon is: {}\n".format(self._pose.longitude))
        #     f.write("heading is: {}\n".format(self._pose.heading))

        ## use dataframes to write localpose and gps data to file
        df_localpose = pd.DataFrame([[local_pose[0], local_pose[1]]], columns=["x", "y"], index=[curr_time])
        self.df_localpose = self.df_localpose.append(df_localpose)
        #self.df_localpose.to_pickle("{}_localpose_data.pkl".format(self.agent_name))

        #df_gps = pd.DataFrame([[self._pose.latitude, self._pose.longitude, self._pose.heading]], columns=["lat", "lon", "heading"], index=[curr_time])
        #self.df_gps = self.df_gps.append(df_gps)
        #self.df_gps.to_pickle("{}_gps_data.pkl".format(self.agent_name))


        self._check_msg.data = 1.0
        self._pub_check.publish(self._check_msg)


    # Converts coordinates to distance and vice versa
    def _normalized_coord(self, coord): # lat/lon x/y specified backwards from stomp (for coord)
        # ros coords are always positive
        normalized_coord = Pose()
        normalized_coord.position.x = (coord.position.y-self._coordSW.longitude)/self._delta_lat
        #y = (coord.position.x-self._coordSW.latitude)/self._delta_lon
        normalized_coord.position.y = (coord.position.x-self._coordSW.latitude)/self._delta_lat
        return normalized_coord

    def _coord_to_dist(self, coord):
        x = (coord.longitude-self._coordSW.longitude)/self._delta_lat
        #y = (coord.longitude-self._coordSW.longitude)/self._delta_lon
        y = (coord.latitude-self._coordSW.latitude)/self._delta_lat
        return [x, y]

    def _dist_to_coord(self, dist):
        coord = GlobalPose()
        coord.latitude = dist[1] * self._delta_lat + self._coordSW.latitude
        #coord.longitude = dist[0] * self._delta_lon + self._coordSW.longitude
        coord.longitude = dist[0] * self._delta_lat + self._coordSW.longitude
        return coord

    # def get_grid(self, msg): # unused
    #     pass

    def valid_pose(self,coord): # check if coord is within boundary
        # lat/lon x/y specified backwards from stomp (for coord) -- separated for easier deugging
        lat = coord.position.x
        lon = coord.position.y
        if ((lat > self._coordNE.latitude) or (lat < self._coordSW.latitude) or
            (lon > self._coordNE.longitude) or (lon < self._coordSW.longitude)):
            return False
        else:
            return True

    def server_ros_erg_control(self):
        s = rospy.Service(
            'RosTacticNorthwesternErgodicControl',
            RosErgodicControl,
            self.handle_ros_erg_control)
        rospy.loginfo_once('Ready to accept hvt/ied positions from ccast')


    def handle_ros_erg_control(self, req):
        """ Receive a HVT/IED positions from CCAST. """

        signal = req.signal

        #These are strings; parse them for index values, use each index to assign each geo_position to the correct distribution type
        str_hvt_indices = req.HVT_indices
        str_ied_indices = req.IED_indices

        # Parameter that tells us if we are using the tanvas? (yes/no)
        str_tanvas = req.tanvas
        if str_tanvas == 'yes' or str_tanvas == 'y' or str_tanvas == 'Yes' or str_tanvas == 'Y':
            self.tanvas = True
        else:
            self.tanvas = False

        # Geo-positions sent from STOMP
        geo_poses = req.geo_positions

        rospy.loginfo('Received %d new geo_positions!', len(geo_poses))
        for index, pose in enumerate(geo_poses, start=1):
            rospy.loginfo('pose %d lat: %f', index, pose.position.x) # lat/lon x/y specified backwards
            rospy.loginfo('pose %d lon: %f', index, pose.position.y) # lat/lon x/y specified backwards
            rospy.loginfo('pose %d alt: %f', index, pose.position.z)

        # Converting strings of indices to lists of indices
        hvt_indices = str_hvt_indices.split(",")
        ied_indices = str_ied_indices.split(",")

        # These lists are to hold the normalized positions of each hvt/ied
        hvt_means = []
        ied_means = []

        # looping through to assign each normalized geo_pose to the correct index
        for idx in hvt_indices:
            if idx == '' or idx == ' ':
                rospy.loginfo("empty string received as hvt position: check your entry")
            else:
                new_idx = int(idx)
                if new_idx > len(geo_poses) - 1:
                    rospy.loginfo("invalid index: out of range of geo_poses list")
                else:
                    if self.valid_pose(geo_poses[new_idx]):
                        hvt_means.append(self._normalized_coord(geo_poses[new_idx]))
                    else:
                        rospy.logwarn('skipping invalid pose %d lat: %f lon: %f', idx, pose.position.x, pose.position.y) # lat/lon x/y specified backwards

        for idx in ied_indices:
            if idx == '' or idx == ' ':
                rospy.loginfo("empty string received as ied position: check your entry")
            else:
                new_idx = int(idx)
                if new_idx > len(geo_poses) - 1:
                    rospy.loginfo("invalid index: out of range of geo_poses list")
                else:
                    if self.valid_pose(geo_poses[new_idx]):
                        ied_means.append(self._normalized_coord(geo_poses[new_idx]))
                    else:
                        rospy.logwarn('skipping invalid pose %d lat: %f lon: %f', idx, pose.position.x, pose.position.y) # lat/lon x/y specified backwards

        # start or stop RosTacticNorthwesternErgodicControl
        if (signal == 'start' and self.tanvas == False):
            #publish hvt_positions
            hvt_msg = PoseArray()
            hvt_msg.poses = hvt_means
            hvt_msg.header.frame_id = 'global'
            self._hvt_publisher.publish(hvt_msg)
            rospy.sleep(1.0) # give it time to process positions

            #publish ied_positions
            ied_msg = PoseArray()
            ied_msg.poses = ied_means
            ied_msg.header.frame_id = 'global'
            self._ied_publisher.publish(ied_msg)
            rospy.sleep(1.0) # give it time to process positions

            # start running controls
            self._control_status = 'run'
            new_control_status = String()
            new_control_status.data = self._control_status
            self._control_status_publisher.publish(new_control_status)

        elif (signal == 'start' and self.tanvas == True):
            #rospy.logwarn('Using the tablet...target distribution will be published to /tanvas_map')
            # start running controls (without publishing hvt's/ied's from here since they will be coming from the tanvas)

            #publish hvt_positions
            hvt_msg = PoseArray()
            hvt_msg.poses = hvt_means
            hvt_msg.header.frame_id = 'global'
            self._tablet_hvt_publisher.publish(hvt_msg)
            rospy.sleep(1.0) # give it time to process positions

            #publish ied_positions
            ied_msg = PoseArray()
            ied_msg.poses = ied_means
            ied_msg.header.frame_id = 'global'
            self._tablet_ied_publisher.publish(ied_msg)
            rospy.sleep(1.0) # give it time to process positions

            self._control_status = 'run'
            new_control_status = String()
            new_control_status.data = self._control_status
            self._control_status_publisher.publish(new_control_status)

        elif (signal == 'stop' or signal == 'Stop'):
            rospy.logwarn('STOP command received')
            self._control_status = 'stop'
            new_control_status = String()
            new_control_status.data = self._control_status
            self._control_status_publisher.publish(new_control_status)

            # cancel all goals
            self.stop_moving_platform()

        else: # signal set to undefined value
            rospy.logwarn('signal arg set to undefined value: %s', signal)


        response = "Service call received %s" % str(req)
        return response

    """
    With the current Pose, this function takes the agent's goal pose, calcuated with the single integrator
    model and controls from the ergodic controller, and converts it to lat/long coordinates.
    Then it uses the position controller to find the goal pose and distance in the body frame.
    """
    def step_rover(self):
        if self._control_status == 'run':
            rospy.loginfo_once("_control_status = run")
            if (self._coordNE is None) or (self._coordSW is None):
                pass
            else:
                next_pose = self.planner_step() # defined in Agent
                coord = self._dist_to_coord(next_pose)
                self._target_pose.latitude = coord.latitude
                self._target_pose.longitude = coord.longitude
                self._target_pose.heading = -1
                # rospy.logwarn("got pose x: %f, y: %f ",next_pose[0], next_pose[1])
                # rospy.logwarn("got pose lat: %f, lon %f ",self._pose.latitude, self._pose.longitude)
                # rospy.logwarn("got target_pose lat: %f, lon %f ",self._target_pose.latitude, self._target_pose.longitude)
                new_goal, distance = self._controller.calculate_new_goal(self._pose, self._target_pose)
                try:
                    # print('new goal pose', new_goal.pose)
                    # print('distance to new goal', distance)
                    # print("Sending pose")
                    # print("(%f,%f)" % (coord.latitude, coord.longitude))
                    # print('next pose', next_pose, 'curr state', self.state)
                    # print('MOVE')
                    self._pub_goal.publish(new_goal)
                    # self.rate.sleep()

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

        elif self._control_status == 'idle':
            rospy.loginfo_once("_control_status = idle")
            #do nothing

        elif self._control_status == 'stop':
            rospy.loginfo_once("_control_status = stop; platform should stop moving")

            # cancel the goals here
            self.stop_moving_platform()

        else:
            rospy.logwarn_once("_control_status set to undefined value: %s", self._control_status)

def shutdown():
    rospy.logwarn('Attempting to cancel move_base goals')
    gv.stop_moving_platform() # cancel current move_base goals
    gv._control_status_publisher.publish(String('stop'))
    rospy.logwarn('clean break')

if __name__ == '__main__':
    # rospy.wait_for_service('goto_position')
    rospy.wait_for_service('/MOVE/make_plan')
    # rospy.loginfo("Got the goto_position service.")
    gv = ControlNode()
    rospy.loginfo("created ControlNode()")

    # start RosTacticNorthwesternErgodicControl service
    gv.server_ros_erg_control()
    rospy.loginfo("created RosTacticNorthwesternErgodicControl service")

    rospy.on_shutdown(shutdown)

    ## create directory for storing rover gps data (NEW: 2022/2/17 - 7:28pm)
    real_path=os.path.realpath(__file__)
    base_path = os.path.dirname(real_path)
    plotting_dir = base_path+"/rt_erg_lib/data/{}_{}_{}_{}_{}_gps_data".format(cyear, cmonth, cday, chour, cmin)
    print("gps plotting_dir for {} is: {}".format(gv.agent_name, plotting_dir))
    if not(os.path.exists(plotting_dir)):
        try:
            os.makedirs(plotting_dir)
        except:
            print("couldn't make gps plotting directory")

    ## counter for saving gps data (we want this to be less than 1 Hz)
    gps_data_counter = 0

    ## save the start time of this experimental run (from rover_0's perspective)
    if (gv.agent_name == "rover_0"):
        start_time_dir = base_path+"/rt_erg_lib/data/"
        with open(start_time_dir+"{}_{}_{}_{}_{}_experiment_start_time.txt".format(cyear, cmonth, cday, chour, cmin), "w") as f:
            f.write("start_time: {}".format(START_TIME))

    try:
        while not rospy.is_shutdown():
            if gv.get_control_status() == 'stop':
                gv.stop_moving_platform() # cancel current move_base goals
                rospy.logwarn_once('Ergodic control tactic stopped')
            elif gv.get_control_status() == 'idle':
                rospy.logwarn_once('Platform is idle')
            elif gv.received_kill_command:
                if gv._pose.latitude > gv.kill_latitude:
                    gv._control_status = "stop"
                gv.received_kill_command = False
            else:
                rospy.logwarn_once('Stepping rover')
                gv.step_rover()

                # publish if we are using the tanvas/tablet (True or False)
                gv.tanvas_status_publisher.publish(gv.tanvas)

                # periodically save rover gps data to the gps directory
                if (gps_data_counter  % 10 == 0):
                    gv.df_localpose.to_pickle("{}/{}_localpose_data.pkl".format(plotting_dir, gv.agent_name))
                    #gv.df_gps.to_pickle("{}/{}_gps_data.pkl".format(plotting_dir, gv.agent_name))
                gps_data_counter = gps_data_counter + 1
            gv.rate.sleep()


    except rospy.ROSInterruptException as e:
        shutdown() # not sure if this one does anything
        print('clean break')
