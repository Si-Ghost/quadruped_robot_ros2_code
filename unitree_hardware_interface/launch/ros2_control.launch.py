import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'unitree_hardware_interface'
    pkg_path = get_package_share_directory(pkg)

    xacro_file = os.path.join(pkg_path, 'description', 'quadruped.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    controller_config = os.path.join(
        pkg_path, 'config', 'quadruped_controllers.yaml')

    record_bag = DeclareLaunchArgument(
        'record_bag', default_value='false',
        description='Record /joint_states to rosbag')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config,
        ],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    bag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=['ros2', 'bag', 'record', '/joint_states',
             '/joint_trajectory_controller/status', '-o', 'motor_bag'],
        output='screen',
    )

    return LaunchDescription([
        record_bag,
        control_node,
        joint_state_spawner,
        trajectory_spawner,
        bag_record,
    ])
