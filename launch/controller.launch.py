import launch
import os
import yaml
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.10", " ",
            "joint_limit_params:=", joint_limit_params, " ",
            "kinematics_params:=", kinematics_params, " ",
            "physical_params:=", physical_params, " ",
            "visual_params:=", visual_params, " ",
            "safety_limits:=", "true", " ",
            "safety_pos_margin:=", "0.15", " ",
            "safety_k_position:=", "20", " ",
            "name:=", "ur", " ",
            "ur_type:=", "ur10e", " ",
            "prefix:=", '""', " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    return robot_description


def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=", "ur", " ",
            "prefix:=", '""', " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic
    
    
def generate_launch_description():
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    config = os.path.join(
        get_package_share_directory('ur10e_moveit2_controller'),
        'config',
        'controller.yaml'
    )
    return launch.LaunchDescription([
        Node(
            package="ur10e_moveit2_controller",
            executable="controller",
            name="controller",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                config,
            ],
        ),
    ])
