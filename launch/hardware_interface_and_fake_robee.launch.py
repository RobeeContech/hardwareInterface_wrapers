# Copyright (c) 2022 RobeeContech, Inc.
# Author: Yoav Fekete

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
  
    # General arguments
    description_package = "robee_hardware_interface"
    description_file = "robee.urdf.xacro"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=",
            "robee",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robee_description"), "rviz", "view_robot.rviz"]
    )


    controller_config = PathJoinSubstitution(
         [FindPackageShare("robee_hardware_interface"), "config", "controller_configuration.yaml"]
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
        "x_y_rot_controller",
        "ptr_controller",
        "joint_state_broadcaster",
        "inclinometer_state_broadcaster",
        "profiler_state_broadcaster",
        "vacuum_ef_controller",
        "glue_ef_controller",
        "tilt_controller",
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
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    fake_robee = Node(
        package="fake_robot",
        executable="fake_robee",
        name="fake_robee",
        output="both",
    )

    nodes_to_start = [
        controller_manager,robot_state_publisher_node,
        fake_robee
        #,rviz_node,
    ] + load_controllers

    return LaunchDescription( nodes_to_start) 
