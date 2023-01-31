# Copyright (c) 2023 RobeeContech, Inc.
# Author: Yoav Fekete

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
  
    # General arguments
    description_package = "hardware_interface_wrappers"
    description_file = "robot.urdf.xacro"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    controller_config = PathJoinSubstitution(
         [FindPackageShare(description_package), "config", "controller_configuration.yaml"]
    )
    
    controller_manager= Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller_config],
            output="both",
        )

    # Load controllers
    load_controllers = []
    for controller in [
        "robee_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description],
    )


    nodes_to_start = [
        controller_manager,robot_state_publisher_node,
    ] + load_controllers

    return LaunchDescription( nodes_to_start) 
