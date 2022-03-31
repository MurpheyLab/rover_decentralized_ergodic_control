#!/usr/bin/env python
from single_integrator import SingleIntegrator
from ergodic_control import RTErgodicControl

# for rendering in rviz
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Pose
import numpy as np
import tf
import rospy

"""
Creates an agent with an associated identifier, model, and planner.
A position controller wraps around the agent's planner to calculate the next states.
"""

class Agent(SingleIntegrator):

    def __init__(self, agent_name,show_in_rviz=False):
        SingleIntegrator.__init__(self)
        self.agent_name = agent_name
        self.show_in_rviz = show_in_rviz
        self.model = SingleIntegrator()

        #numbasis was originally 10 here; (2022/2/1: if I change num_basis, I need to change it in the touchscreen code too)
        #original: batch_size=100, horizon=15
        self.planner = RTErgodicControl(self.agent_name, self.model,
                                        horizon=15, num_basis=15, batch_size=100)
        self.reset() # this is overwritten with the GPS call


        # rendering in rviz
        if self.show_in_rviz:
            num_pts = rospy.get_param("/control_node/pts")
            self.scale = 10. # scale up for rviz simulation
            self.__br = tf.TransformBroadcaster()
            self._target_dist_pub = rospy.Publisher(agent_name+'/target_dist', GridMap, queue_size=1)

            gridmap = GridMap()
            arr = Float32MultiArray()
            arr.data = self.planner.target_dist.grid_vals[:-1]
            arr.layout.dim.append(MultiArrayDimension())
            arr.layout.dim.append(MultiArrayDimension())

            arr.layout.dim[0].label="column_index"
            arr.layout.dim[0].size=self.planner.target_dist.num_pts[0]
            arr.layout.dim[0].stride=np.product(self.planner.target_dist.num_pts)
            arr.layout.dim[1].label="row_index"
            arr.layout.dim[1].size=self.planner.target_dist.num_pts[1]
            arr.layout.dim[1].stride=self.planner.target_dist.num_pts[1]

            gridmap.layers.append("elevation")
            gridmap.data.append(arr)
            gridmap.info.length_x=self.planner.basis.dl[0]*self.scale
            gridmap.info.length_y=self.planner.basis.dl[1]*self.scale
            gridmap.info.pose.position.x=self.planner.basis.dl[0]*self.scale/2.
            gridmap.info.pose.position.y=self.planner.basis.dl[1]*self.scale/2.
            gridmap.info.header.frame_id = "world"
            gridmap.info.resolution = self.scale/num_pts

            self._grid_msg = gridmap


    def planner_step(self):
        """The idea here is to use the agent as a planner to forward
        plan where we want the rover to go. The rover will use a
        position controller to wrap around the planner."""
        ctrl = self.planner(self.state) # calculates controls from the Ergodic Controller

        # rendering in rviz
        if self.show_in_rviz:
            # update agent location
            self.__br.sendTransform( # sends update for rendering in RVIZ
                (self.state[0]*self.scale, self.state[1]*self.scale, 0.),
                (0.,0.,0.,1.),
                rospy.Time.now(),
                self.agent_name,
                "world"
            )

            # update target distribution
            grid_vals =  self.planner.target_dist.grid_vals

            self._grid_msg.data[0].data = grid_vals[:-1]
            self._target_dist_pub.publish(self._grid_msg)

        return self.step(ctrl) # finds the next states from the dynamics defined in SingleIntegrator
