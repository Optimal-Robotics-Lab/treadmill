from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the 'serial_port' argument
    # Default is an empty string, which triggers the node's auto-detection logic
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="",
        description="Path to the USB serial device (e.g. /dev/ttyUSB0). Leave empty to auto-detect.",
    )

    # Define the treadmill node
    treadmill_node = Node(
        package="treadmill_control",  # <--- REPLACE with your actual package name
        executable="treadmill_node",  # <--- REPLACE with your entry point name in setup.py
        name="treadmill",
        output="screen",
        emulate_tty=True,
        parameters=[{"serial_port": LaunchConfiguration("serial_port")}],
    )

    return LaunchDescription([serial_port_arg, treadmill_node])
