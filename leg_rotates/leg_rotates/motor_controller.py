import rclpy
from rclpy.node import Node
from interface_type.msg import MotorCommand, MotorState, MotorPd

from unitree_actuator_sdk import *

TOTAL_MOTORS = 12
INIT_TIMEOUT_ITER = 50


class QuinticTrajectory:
    """五次多项式轨迹，满足 q(0)=q0, q(T)=qdes, dq(0)=dq(T)=0, ddq(0)=ddq(T)=0"""

    def __init__(self, q0, q_des, duration):
        self.q0 = float(q0)
        self.q_des = float(q_des)
        self.T = float(duration)
        delta = self.q_des - self.q0
        T3 = self.T ** 3
        T4 = self.T ** 4
        T5 = self.T ** 5
        self.a0 = self.q0
        self.a3 = 10.0 * delta / T3
        self.a4 = -15.0 * delta / T4
        self.a5 = 6.0 * delta / T5

    def evaluate(self, t):
        """返回 (位置, 速度)，t 为轨迹已流逝时间（秒）"""
        if t <= 0.0:
            return self.q0, 0.0
        if t >= self.T:
            return self.q_des, 0.0

        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t

        q = self.a0 + self.a3 * t3 + self.a4 * t4 + self.a5 * t5
        dq = 3.0 * self.a3 * t2 + 4.0 * self.a4 * t3 + 5.0 * self.a5 * t4
        return q, dq


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info('MotorControllerNode started')

        self.declare_parameter('trajectory_duration', 1.0)
        self.traj_duration = self.get_parameter('trajectory_duration').value

        self.cmd_pub = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.pd_pub = self.create_publisher(MotorPd, 'motor_pd', 10)
        self.state_sub = self.create_subscription(
            MotorState, 'motor_state', self.state_callback, 10)

        self.gear_ratio = queryGearRatio(MotorType.GO_M8010_6)
        deg_to_raw = 3.14159 / 180.0 * self.gear_ratio
        # 每个电机的目标角度（度），直接用 target_degrees[i] = 90 赋值即可
        self.target_degrees = [-10.0] * TOTAL_MOTORS
        self.target_offsets = [d * deg_to_raw for d in self.target_degrees]

        self.q_ini = [None] * TOTAL_MOTORS
        self.trajectories = [None] * TOTAL_MOTORS
        self.init_counter = 0
        self.init_done = False
        self.traj_start_time = None
        self._valid_readings = [0] * TOTAL_MOTORS
        self._min_valid_readings = 3
        self._last_display_time = 0.0
        self._display_interval = 0.5

        self.publish_pd()
        self.pd_timer = self.create_timer(1.0, self.publish_pd)
        self.control_timer = self.create_timer(0.02, self.control_loop)

    def publish_pd(self):
        pd_msg = MotorPd()
        pd_msg.kp = [0.4] * TOTAL_MOTORS
        pd_msg.kd = [0.01] * TOTAL_MOTORS
        self.pd_pub.publish(pd_msg)

    def state_callback(self, msg):
        if not self.init_done:
            for i in range(TOTAL_MOTORS):
                if msg.merror[i] != -1:
                    if self.q_ini[i] is None:
                        self._valid_readings[i] += 1
                        if self._valid_readings[i] >= self._min_valid_readings:
                            self.q_ini[i] = msg.q[i]
                            self.get_logger().info(
                                f'  Motor {i:02d} init q={self.q_ini[i]:.3f} '
                                f'(after {self._min_valid_readings} valid readings)')

        self.update_display(msg)

    def update_display(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_display_time < self._display_interval:
            return
        self._last_display_time = now

        import shutil
        term_w = shutil.get_terminal_size((120, 24)).columns

        ratio = self.gear_ratio
        active = [i for i in range(TOTAL_MOTORS) if msg.merror[i] != -1]
        lines = []
        for i in active:
            traj = self.trajectories[i]
            if traj is not None:
                elapsed = (self.get_clock().now().nanoseconds * 1e-9
                           - (self.traj_start_time or 0))
                remain = max(0, traj.T - elapsed)
                prog = f'{remain:.1f}s'
            elif self.q_ini[i] is not None:
                prog = 'hold'
            else:
                prog = '----'
            line = (f" M{i:02d} tau={msg.tau[i]:+6.2f} "
                    f"q={msg.q[i]/ratio:+7.3f} dq={msg.dq[i]/ratio:+7.3f} "
                    f"T={msg.temp[i]:4.0f} err={msg.merror[i]:2d} [{prog:>8}]")
            lines.append(line[:term_w - 1])

        lines.append(f'--- {len(active)}/{TOTAL_MOTORS} motors ---')

        if not getattr(self, '_display_ready', False):
            print('\n' * (len(lines) - 1))
            self._display_ready = True

        print(f"\033[{len(lines)}A", end='')
        for line in lines:
            print(f"\033[K{line}")
        print('\033[J', end='', flush=True)

    def _start_trajectories(self, now):
        """为所有在线的电机创建五次多项式轨迹"""
        for i in range(TOTAL_MOTORS):
            if self.q_ini[i] is not None:
                q_start = self.q_ini[i]
                q_end = self.q_ini[i] + self.target_offsets[i]
                self.trajectories[i] = QuinticTrajectory(
                    q_start, q_end, self.traj_duration)
                self.get_logger().info(
                    f'  M{i:02d} traj: {q_start:.1f} -> {q_end:.1f} '
                    f'({self.traj_duration:.1f}s)')
        self.traj_start_time = now

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self.init_done:
            self.init_counter += 1
            all_ready = all(q is not None for q in self.q_ini)
            timed_out = self.init_counter >= INIT_TIMEOUT_ITER

            if all_ready or timed_out:
                self.init_done = True
                active = sum(1 for q in self.q_ini if q is not None)
                self.get_logger().info(
                    f'Init complete: {active}/{TOTAL_MOTORS} motors, '
                    f'trajectory {self.traj_duration:.1f}s')
                self._start_trajectories(now)

        cmd = MotorCommand()
        cmd.id = [i % 3 for i in range(TOTAL_MOTORS)]
        cmd.q = [0.0] * TOTAL_MOTORS
        cmd.dq = [0.0] * TOTAL_MOTORS
        cmd.tau = [0.0] * TOTAL_MOTORS

        if self.init_done and self.traj_start_time is not None:
            elapsed = now - self.traj_start_time

            for i in range(TOTAL_MOTORS):
                traj = self.trajectories[i]
                if traj is None:
                    continue
                q, dq = traj.evaluate(elapsed)
                cmd.q[i] = q
                cmd.dq[i] = dq

        self.cmd_pub.publish(cmd)

    def __del__(self):
        cmd = MotorCommand()
        cmd.id = [i % 3 for i in range(TOTAL_MOTORS)]
        cmd.q = [0.0] * TOTAL_MOTORS
        cmd.dq = [0.0] * TOTAL_MOTORS
        cmd.tau = [0.0] * TOTAL_MOTORS
        try:
            self.cmd_pub.publish(cmd)
        except Exception:
            pass
        self.get_logger().info('Controller stopped')


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
