from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command

from launch_ros.actions import Node


def generate_launch_description():

    # Directories
    pkg_ros2_workshop = get_package_share_directory("ros2_workshop")
    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")

    sdf_file = PathJoinSubstitution([pkg_ros2_workshop, "sdf", "visualize_lidar.sdf"])
    xacro_file = PathJoinSubstitution([pkg_ros2_workshop, "urdf", "robot.urdf.xacro"])

    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py"]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {
                "robot_description": Command(
                    ["xacro", " ", xacro_file, " ", "gazebo:=ignition"]
                )
            },
        ],
    )

    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            (
                "ign_args",
                [sdf_file, " -v 4", " -r"],
            )
        ],
    )

    # Bridges
    cmd_vel_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="cmd_vel_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"
        ],
        remappings=[
            (
                "model/vehicle_blue/cmd_vel",
                "cmd_vel",
            )
        ],
    )

    odom_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="odom_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"
        ],
        remappings=[
            (
                "model/vehicle_blue/odometry",
                "odom",
            )
        ],
    )

    scan_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="scan_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"],
        remappings=[
            (
                "lidar2",
                "scan",
            )
        ],
    )

    odom_tf_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="odom_tf_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
        ],
        remappings=[
            (
                "model/vehicle_blue/tf",
                "tf",
            )
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ignition_gazebo,
            cmd_vel_bridge,
            odom_bridge,
            scan_bridge,
            odom_tf_bridge,
        ]
    )
