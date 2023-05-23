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
    pkg_pcd_demo = get_package_share_directory('pcd_demo')

    moveit_launch = PathJoinSubstitution([pkg_ur_bringup, 'launch', 'ur_control.launch.py'])
    rviz_launch = PathJoinSubstitution([pkg_ur_bringup, 'launch', 'ur_moveit.launch.py'])
    ur10e_moveit2_controller_launch = PathJoinSubstitution([pkg_ur10e_moveit2_controller, 'launch', 'controller.launch.py'])
    pcd_to_ply_launch = PathJoinSubstitution([pkg_pcd_demo, 'launch', 'pcd_to_ply_pause.launch.py'])

    tf_static = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     arguments=['0.0', '0.0', '0.0', '0.0', '-1.57', '0.0', 'tool0', 'depth_camera/link/depth_camera1'])

    sim_camera_adapter = Node(package='ur10e_moveit2_controller',
                              executable='sim_camera_adapter',
                              arguments=[])

    pcd_bridge = Node(package='ros_ign_bridge',
                      executable='parameter_bridge',
                      arguments=['/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'])

    pose_bridge = Node(package='ros_ign_bridge',
                       executable='parameter_bridge',
                       arguments=['/depth_camera/pose@geometry_msgs/msg/Pose]ignition.msgs.Pose'])

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

    pcd_to_ply = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([pcd_to_ply_launch]),
        launch_arguments=[]
    )

    ld = launch.LaunchDescription()
    ld.add_action(moveit)
    ld.add_action(rviz)
    ld.add_action(tf_static)
    ld.add_action(sim_camera_adapter)
    ld.add_action(pcd_bridge)
    ld.add_action(pose_bridge)
    ld.add_action(pcd_to_ply)

    return ld
