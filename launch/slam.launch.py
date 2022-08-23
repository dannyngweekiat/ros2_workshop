from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    # Directories
    pkg_ros2_workshop = get_package_share_directory("ros2_workshop")

    slam_config_file = PathJoinSubstitution(
        [
            pkg_ros2_workshop,
            "config",
            "mapper_params_online_async.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            pkg_ros2_workshop,
            "config",
            "rviz.mapping.rviz",
        ]
    )

    ros2_workshop_ign_launch = PathJoinSubstitution(
        [pkg_ros2_workshop, "launch", "ignition.launch.py"]
    )

    # ROS Ign launch
    ros2_workshop_ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros2_workshop_ign_launch])
    )

    start_async_slam_toolbox_node = Node(
        parameters=[slam_config_file, {"use_sim_time": True}],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [ros2_workshop_ignition_launch, start_async_slam_toolbox_node, rviz_node]
    )
