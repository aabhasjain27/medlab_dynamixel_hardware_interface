import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory("medlab_dynamixel_hardware_interface")
    default_config = os.path.join(pkg_share, "config", "config_arm_0.yaml")

    config_arg = DeclareLaunchArgument("config", default_value=default_config)
    namespace_arg = DeclareLaunchArgument("namespace", default_value="dynamixel")

    node = Node(
        package="medlab_dynamixel_hardware_interface",
        executable="dynamixel_hardware_interface",
        name="dynamixels",
        namespace=LaunchConfiguration("namespace"),
        parameters=[LaunchConfiguration("config")],
        output="screen",
    )

    return LaunchDescription([
        config_arg,
        namespace_arg,
        node,
    ])
