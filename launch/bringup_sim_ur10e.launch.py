import launch
import launch.actions
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import subprocess


def generate_launch_description():
    pkg_ur_bringup = get_package_share_directory('ur_bringup')
    pkg_ur10e_moveit2_controller = get_package_share_directory('ur10e_moveit2_controller')

    moveit_launch = PathJoinSubstitution([pkg_ur_bringup, 'launch', 'ur_control.launch.py'])
    rviz_launch = PathJoinSubstitution([pkg_ur_bringup, 'launch', 'ur_moveit.launch.py'])
    ur10e_moveit2_controller_launch = PathJoinSubstitution([pkg_ur10e_moveit2_controller, 'launch', 'controller.launch.py'])

    tf_static = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     arguments=['0.0', '0.0', '0.0', '0.0', '-1.57', '0.0', 'tool0', 'depth_camera/link/depth_camera1'])

    sim_camera_adapter = Node(package='ur10e_moveit2_controller',
                              executable='sim_camera_adapter',
                              arguments=[])

    moveit = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([moveit_launch]),
        launch_arguments={'ur_type': 'ur10e',
                          'robot_ip': 'yyy.yyy.yyy.yyy',
                          'use_fake_hardware': 'true',
                          'launch_rviz': 'false',
                          'initial_joint_controller': 'joint_trajectory_controller'}.items()
    )
    rviz = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments={'ur_type': 'ur10e',
                          'robot_ip': 'xxx.xxx',
                          'use_fake_hardware': 'true',
                          'launch_rviz': 'true'}.items()
    )
    controller = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([ur10e_moveit2_controller_launch]),
        launch_arguments=[]
    )

    ld = launch.LaunchDescription()
    ld.add_action(moveit)
    ld.add_action(rviz)
    ld.add_action(tf_static)
    ld.add_action(sim_camera_adapter)

    return ld
