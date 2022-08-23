from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    # Directories
    pkg_ros2_workshop = get_package_share_directory("ros2_workshop")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    rviz_config_file = PathJoinSubstitution(
        [
            pkg_ros2_workshop,
            "config",
            "rviz.nav.rviz",
        ]
    )

    nav2_config_file = PathJoinSubstitution(
        [
            pkg_ros2_workshop,
            "config",
            "nav2_params.yaml",
        ]
    )

    map_file = PathJoinSubstitution(
        [
            pkg_ros2_workshop,
            "map",
            "map.yaml",
        ]
    )

    ros2_workshop_ign_launch = PathJoinSubstitution(
        [pkg_ros2_workshop, "launch", "ignition.launch.py"]
    )
    nav2_bringup_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, "launch", "bringup_launch.py"]
    )

    # ROS Ign launch
    ros2_workshop_ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros2_workshop_ign_launch])
    )
    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch]),
        launch_arguments=[
            ("params_file", nav2_config_file),
            ("map", map_file),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            ros2_workshop_ignition_launch,
            rviz_node,
            nav2_bringup,
        ]
    )
