#!/usr/bin/env python3
"""
给 JointTrajectoryController 发目标角度。
直接改下面三个参数，然后运行：
  python3 send_trajectory.py
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

# ============================================================
#  要改的就这里
# ============================================================
RELATIVE = True           # True: 从当前位置相对偏移, False: 绝对角度
TARGET_DEGREES = [
    -10,   # joint_0
    -10,   # joint_1
    -10,   # joint_2
    -10,   # joint_3
    -10,   # joint_4
    -10,   # joint_5
    -10,   # joint_6
    -10,   # joint_7
    -10,   # joint_8
    -10,   # joint_9
    -10,   # joint_10
    -10,   # joint_11
]
DURATION = 1.0  # 秒
# ============================================================

JOINT_NAMES = [f'joint_{i}' for i in range(12)]


def get_current_positions(node, timeout_sec=3.0):
    """读一次 /joint_states，返回 12 个关节的当前位置（弧度）"""
    positions = [None] * 12

    def cb(msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name.startswith('joint_') and name[6:].isdigit():
                idx = int(name[6:])
                if 0 <= idx < 12:
                    positions[idx] = pos

    sub = node.create_subscription(JointState, '/joint_states', cb, 10)

    waited = 0.0
    while rclpy.ok() and waited < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        waited += 0.1
        if all(p is not None for p in positions):
            break

    node.destroy_subscription(sub)
    return positions


def main():
    rclpy.init()
    node = Node('send_trajectory')

    pos_deg = TARGET_DEGREES.copy()

    if RELATIVE:
        node.get_logger().info('读取当前角度...')
        current = get_current_positions(node)
        for i in range(12):
            if current[i] is not None:
                pos_deg[i] = math.degrees(current[i]) + TARGET_DEGREES[i]
            else:
                node.get_logger().warn(f'joint_{i} 没读到，按绝对角度 {pos_deg[i]} 发送')
        node.get_logger().info(f'当前: {[round(math.degrees(c), 1) if c else None for c in current]}')
        node.get_logger().info(f'偏移: {TARGET_DEGREES}')
        node.get_logger().info(f'目标: {[round(d, 1) for d in pos_deg]}')

    client = ActionClient(node, FollowJointTrajectory,
                          '/joint_trajectory_controller/follow_joint_trajectory')

    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('控制器没连上，先确认 launch 启动了')
        node.destroy_node()
        rclpy.shutdown()
        return

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_NAMES

    point = JointTrajectoryPoint()
    point.positions = [math.radians(d) for d in pos_deg]
    point.time_from_start.sec = int(DURATION)
    point.time_from_start.nanosec = int((DURATION - int(DURATION)) * 1e9)
    goal.trajectory.points = [point]

    node.get_logger().info(f'发送: {[round(d, 1) for d in pos_deg]} 度, {DURATION}s')
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result and result.accepted:
        node.get_logger().info('目标已接受')
    else:
        node.get_logger().error('目标被拒绝')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
