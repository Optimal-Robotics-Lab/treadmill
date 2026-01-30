import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Define the package name
    package_name = "treadmill_gui"

    # GUI Node configuration
    treadmill_gui_node = Node(
        package=package_name,
        executable="treadmill_gui_node",
        name="treadmill_gui",
        output="screen",
        parameters=[
            # You can add a parameters file here if needed:
            # os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')
        ],
        remappings=[
            # Example: ('treadmill/status', 'hardware/treadmill_status')
        ],
    )

    return LaunchDescription([treadmill_gui_node])
