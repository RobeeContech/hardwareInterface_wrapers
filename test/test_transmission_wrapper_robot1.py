# Copyright 2022, RobeeContech LTD
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILI10TY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import unittest
import os
import time
import pytest

import launch_testing
from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import JointState

@pytest.mark.launch_test
def generate_test_description():
   
    dir_path = os.path.dirname(os.path.realpath(__file__))

    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dir_path, "/../launch/transmission_wrapper_robot1.test.launch.py"]),
    )

    ld = []
    ld += [launch_testing.actions.ReadyToTest(), launch_file]

    return LaunchDescription(ld)


class RobeeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("transmission_wrapper_test_node")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def topic_callback(cls,position, velocity):
        cls.position= position
        cls.velocity= velocity

    def init_robot(self):
        # subscribe to joint states to validate the path executed
        self.subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
             lambda msg:  self.topic_callback(msg.position,msg.velocity),
            10)
        self.subscription  # prevent unused variable warning

        # test action appearance
        self.jtc_action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/joint_controller/follow_joint_trajectory"
        )
        if self.jtc_action_client.wait_for_server(10) is False:
            raise Exception(
                "Could not reach /joint_controller/follow_joint_trajectory action server,"
                "make sure that controller is active (load + start)"
            )

    def call_action(self, ac_client, g):
        future = ac_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            return future.result()
        else:
            self.node.get_logger().info("Exception while calling action")
            raise Exception(f"Exception while calling action: {future.exception()}")

    def get_result(self, ac_client, goal_response):

        future_res = ac_client._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self.node, future_res)
        if future_res.result() is not None:
            return future_res.result().result
        else:
            raise Exception(f"Exception while calling action: {future_res.exception()}")

    def test_trajectory(self):
        """Test robot movement."""
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "joint1"
        ]
        position_list = [[0.0]]
        position_list.append([0.5])
        position_list.append([1.0])


        velocity_list = [[0.1]]
        velocity_list.append([0.2])
        velocity_list.append([0.3])
        
        duration_list = [
            Duration(sec=1, nanosec=0),
            Duration(sec=2, nanosec=0),
            Duration(sec=3, nanosec=0),
        ]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.velocities = velocity_list[i]
            point.time_from_start = duration_list[i]
            goal.trajectory.points.append(point)

        self.node.get_logger().info("Sending simple goal")

        goal_response = self.call_action(self.jtc_action_client, goal)
        n = 3
        while n>0 and not goal_response.accepted:
            self.node.get_logger().info("Sleep")
            time.sleep(0.5)
            goal_response = self.call_action(self.jtc_action_client, goal)
            n= n-1
        
        self.assertEqual(goal_response.accepted, True)
        self.node.get_logger().info("goal accepted")

        if goal_response.accepted:
            result = self.get_result(self.jtc_action_client, goal_response)
            self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
            self.node.get_logger().info("Received result SUCCESSFUL")           
            w = 10
            self.node.get_logger().info('w =  "%d"' % w)
            while  w>0:
                rclpy.spin_once(self.node)
                time.sleep(0.5)
                w= w-1
            self.node.get_logger().info('Last position "%s"' % self.position)
            self.assertEqual(self.position[0], 1.0)
            self.assertEqual(self.velocity[0], 0.3)



            