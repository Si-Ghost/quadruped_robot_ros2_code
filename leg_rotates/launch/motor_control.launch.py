from launch import LaunchDescription
from launch.actions import ExecuteProcess
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

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/motor_state', '-o', 'motor_bag'],
        output='screen',
    )

    return LaunchDescription([
        motor_rotates_node,
        motor_controller_node,
        bag_record,
    ])
