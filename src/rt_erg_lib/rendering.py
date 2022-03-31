#!/usr/bin/env python
import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose, Point
from std_msgs.msg import Empty
import tf
from tf import transformations as trans

class QuadVisual(object):
    '''
    Renders agents and paths in RVIZ. Agent locations are based only on the
    "current step" and paths only represent previously visited locations
    (no forward projection of planned path). A single instance of this node may
    be used to render the entire swarm (even if nodes update asynchronously).

    Listens to tf.TransformListener to get agent locations from ROS
    (broadcast by agent.py)
    '''
    def __init__(self, scale=0.1):

        rospy.init_node("agent_rendering")

        agent_id = rospy.get_param('~agent_id', default='rover')

        print("agent_id is: ")
        print(agent_id)
        
        agent_names = [agent_id]
        self._agent_names    = agent_names
        self._scale          = scale

        self._agent_markers = MarkerArray()
        self._markers = [
            Marker() for i in range(len(agent_names))
        ]
        self._path_markers = [
            Marker() for i in range(len(agent_names))
        ]
        self._agent_markers.markers = self._markers + self._path_markers

        # instantiatiate the publishers
        self._marker_pub = rospy.Publisher('agent/visual', MarkerArray, queue_size=1)

        self.__build_rendering()
        self.listener = tf.TransformListener()

        self._rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            self.update_rendering()
            self._rate.sleep()

    def update_rendering(self):
        for agent_name, marker, line_m in zip(self._agent_names, self._markers, self._path_markers):
            try:
                (trans, rot) = self.listener.lookupTransform(
                    "world", agent_name, rospy.Time(0)
                )
                marker.pose.position.x = trans[0]
                marker.pose.position.y = trans[1]

                marker.pose.orientation.x = rot[0]
                marker.pose.orientation.y = rot[1]
                marker.pose.orientation.z = rot[2]
                marker.pose.orientation.w = rot[3]

                line_m.points.append(
                    Point(marker.pose.position.x, marker.pose.position.y,
                          0.1))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self._marker_pub.publish(self._agent_markers)

    def __build_rendering(self):

        i=0
        for name, agent_marker, path_marker in zip(self._agent_names, self._markers,self._path_markers):
            rgb = np.random.uniform(0,1, size=(3,))
            quad_scale = 1 # incrased size for grid
            agent_marker.header.frame_id = "world"
            agent_marker.header.stamp = rospy.Time(0)
            agent_marker.ns = name
            agent_marker.id = 0
            agent_marker.action = Marker.ADD
            agent_marker.scale.x = quad_scale
            agent_marker.scale.y = quad_scale
            agent_marker.scale.z = quad_scale
            agent_marker.color.a = 1.0
            # uncomment if you want colors to be randomly assigned
            # agent_marker.color.r = rgb[0]
            # agent_marker.color.g = rgb[1]
            # agent_marker.color.b = rgb[2]
            # custom color for numerals
            # comment out if you want colors to be randomly assigned
            agent_marker.color.r = 0.
            agent_marker.color.g = 0.
            agent_marker.color.b = 0.
            agent_marker.pose.position.z = np.random.uniform(1.6,4)
            agent_marker.type = Marker.CUBE

            # Make Trajectory Lines
            line_scale = 0.1
            path_marker.header.frame_id = "world"
            path_marker.header.stamp = rospy.Time(0)
            path_marker.ns = name+'/path'
            path_marker.id = i
            path_marker.action = Marker.ADD
            path_marker.scale.x = line_scale
            path_marker.color = agent_marker.color
            path_marker.color.a = 0.7
            path_marker.pose.position.z = 0.1
            path_marker.type = Marker.LINE_STRIP
            path_marker.pose.orientation = agent_marker.pose.orientation # added to remove error messages in RVIZ

            i+=1

if __name__ == '__main__':
    agent_rendering = QuadVisual()
    try:
        agent_rendering.run()
    except rospy.ROSInterruptException:
        pass
