from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wur_osu_apples23',
            name='measurement_listener',
            executable='measurement_listener'
        ),
        Node(
            package='wur_osu_apples23',
            name='heuristic_controller',
            executable='heuristic_controller'
        ),
        Node(
            package='wur_osu_apples23',
            name='pull_twist_controller',
            executable='pull_twist_controller'
        )
    ])
