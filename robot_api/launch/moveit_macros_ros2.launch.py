import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mobipick", package_name="mobipick_moveit2_config")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/mobipick.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    ns = LaunchConfiguration("namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")
    prefix = LaunchConfiguration("prefix")
    
    node = Node(
        package="robot_api",
        executable="moveit_macros_ros2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"planning_scene_config_file": os.path.join(get_package_share_directory("grasplan_core"), "config", "cic_planning_scene.yaml")},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace for all nodes"
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value=ns,
            description="tf_prefix to be used"
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value=f"{tf_prefix}/" if tf_prefix != "" else "",
            description="Prefix used in all config files"
        ),
        GroupAction([PushROSNamespace(ns), node,]),
    ])