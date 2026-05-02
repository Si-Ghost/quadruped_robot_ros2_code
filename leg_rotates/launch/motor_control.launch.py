from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_rotates_node = Node(
        package='leg_rotates',
        executable='motor_rotates',
        name='motor_rotates',
        output='screen',
    )

    motor_controller_node = Node(
        package='leg_rotates',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{'trajectory_duration': 1.0}],
    )

    return LaunchDescription([
        motor_rotates_node,
        motor_controller_node,
    ])
