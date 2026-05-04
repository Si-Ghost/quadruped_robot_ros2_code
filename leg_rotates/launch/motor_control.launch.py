from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_rotates_node = Node(
        package='leg_rotates',
        executable='motor_rotates',
        name='motor_rotates',
        output='screen',
        parameters=[{'trajectory_duration': 1.0}],
    )

    return LaunchDescription([
        motor_rotates_node,
    ])
