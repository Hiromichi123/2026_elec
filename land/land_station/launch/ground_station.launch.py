from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="land_station",
            executable="ground_station_ui",
            name="ground_station_ui",
            output="screen",
        )
    ])
