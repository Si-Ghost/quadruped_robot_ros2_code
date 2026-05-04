import os
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from interface_type.msg import MotorState, MotorPd

from unitree_actuator_sdk import *

PORTS = [
    {'path': '/dev/ttyUSB0', 'base': 0},
    {'path': '/dev/ttyUSB1', 'base': 3},
    {'path': '/dev/ttyUSB2', 'base': 6},
    {'path': '/dev/ttyUSB3', 'base': 9},
]
MOTORS_PER_PORT = 3
TOTAL_MOTORS = 12


class QuinticTrajectory:

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


class MotorRotatesNode(Node):
    def __init__(self):
        super().__init__('motor_rotates')
        self.get_logger().info('MotorRotatesNode starting')

        self.motor_mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        self.gear_ratio = queryGearRatio(MotorType.GO_M8010_6)
        self._devnull = os.open(os.devnull, os.O_WRONLY)

        # ── 串口 ──────────────────────────────────────────────
        self.ports = []
        for cfg in PORTS:
            port = {'path': cfg['path'], 'base': cfg['base'],
                    'serial': None, 'cmd': None, 'data': None,
                    'active': False, 'failures': 0, 'suspended': False,
                    'probe_cycle': 0}
            try:
                port['serial'] = SerialPort(cfg['path'])
                port['cmd'] = MotorCmd()
                port['data'] = MotorData()
                port['cmd'].motorType = MotorType.GO_M8010_6
                port['data'].motorType = MotorType.GO_M8010_6
                port['active'] = True
                self.get_logger().info(
                    f'  {cfg["path"]} -> motors {cfg["base"]}-{cfg["base"]+2}')
            except Exception as e:
                self.get_logger().warn(f'  {cfg["path"]} unavailable: {e}')
            self.ports.append(port)

        if not any(p['active'] for p in self.ports):
            self.get_logger().error('No serial ports available')

        # ── 增益 ──────────────────────────────────────────────
        self.kp = [0.0] * TOTAL_MOTORS    # 初始化期间 kp=kd=0，电机不动
        self.kd = [0.0] * TOTAL_MOTORS
        self._kp_configured = 0.4
        self._kd_configured = 0.01

        # ── 轨迹参数 ──────────────────────────────────────────
        self.declare_parameter('trajectory_duration', 1.0)
        self.traj_duration = self.get_parameter('trajectory_duration').value
        deg_to_raw = 3.14159 / 180.0 * self.gear_ratio
        self.target_degrees = [-10.0] * TOTAL_MOTORS
        self.target_offsets = [d * deg_to_raw for d in self.target_degrees]

        # ── 状态 ──────────────────────────────────────────────
        self.q_ini = [None] * TOTAL_MOTORS
        self.trajectories = [None] * TOTAL_MOTORS
        self._valid_readings = [0] * TOTAL_MOTORS
        self._min_valid_readings = 3
        self.init_done = False
        self.traj_start_time = None
        self.seen = [False] * TOTAL_MOTORS

        # ── 健康 ──────────────────────────────────────────────
        self._cycle_count = 0
        self._suspend_after = 10
        self._probe_every = 100

        # ── 显示 ──────────────────────────────────────────────
        self._display_ready = False
        self._last_display = 0.0

        # ── ROS 接口 ──────────────────────────────────────────
        self.pd_sub = self.create_subscription(
            MotorPd, 'motor_pd', self.pd_callback, 10)
        self.state_pub = self.create_publisher(
            MotorState, 'motor_state', 10)
        self.control_timer = self.create_timer(0.02, self.tick)

        # 发布初始增益
        self._publish_pd()

    # ── ROS callbacks ──────────────────────────────────────────

    def pd_callback(self, msg):
        self.kp = list(msg.kp)
        self.kd = list(msg.kd)

    def _publish_pd(self):
        # 已合并到单节点，不再需要 topic 发增益
        # 但保留这个接口方便将来外部调参
        pass

    # ── 主循环 ────────────────────────────────────────────────

    def tick(self):
        self._cycle_count += 1
        state = self._empty_state()
        now = self.get_clock().now().nanoseconds * 1e-9

        # 当前周期的指令值
        if self.traj_start_time is not None:
            elapsed = now - self.traj_start_time
        else:
            elapsed = 0.0

        # ── step 1: 通信 ────────────────────────────────────
        for port in self.ports:
            if not port['active']:
                continue
            if self._skip_port(port):
                continue

            alive = 0
            for local_id in range(MOTORS_PER_PORT):
                gi = port['base'] + local_id

                # 确定本周期发给这个电机的指令
                traj = self.trajectories[gi]
                if traj is not None:
                    q, dq = traj.evaluate(elapsed)
                else:
                    q, dq = 0.0, 0.0

                r = self._sendrecv_one(port, local_id, gi, q, dq,
                                       self.kp[gi], self.kd[gi])
                if r is not None:
                    state.tau[gi] = r['tau']
                    state.q[gi] = r['q']
                    state.dq[gi] = r['dq']
                    state.temp[gi] = r['temp']
                    state.merror[gi] = r['merror']
                    self.seen[gi] = True
                    alive += 1

            self._update_health(port, alive > 0)

        # ── step 2: 采集初始位置 ────────────────────────────
        if not self.init_done:
            for i in range(TOTAL_MOTORS):
                if state.merror[i] != -1 and self.q_ini[i] is None:
                    self._valid_readings[i] += 1
                    if self._valid_readings[i] >= self._min_valid_readings:
                        self.q_ini[i] = state.q[i]
                        self.get_logger().info(
                            f'  Motor {i:02d} init q={self.q_ini[i]:.3f} '
                            f'(after {self._min_valid_readings} valid readings)')
            if any(q is not None for q in self.q_ini):
                self._on_init_done()

        # ── step 3: 发布 + 显示 ──────────────────────────────
        self.state_pub.publish(state)
        self._update_display(state)

    # ── 通信 ──────────────────────────────────────────────────

    def _sendrecv_one(self, port, local_id, gi, q, dq, kp, kd):
        port['cmd'].id = local_id
        port['cmd'].mode = self.motor_mode
        port['cmd'].q = q
        port['cmd'].dq = dq
        port['cmd'].tau = 0.0
        port['cmd'].kp = kp
        port['cmd'].kd = kd

        with self._quiet_sdk():
            ok = port['serial'].sendRecv(port['cmd'], port['data'])
        if ok and port['data'].correct:
            d = port['data']
            return {'q': float(d.q), 'dq': float(d.dq), 'tau': float(d.tau),
                    'temp': float(d.temp), 'merror': int(d.merror)}
        return None

    # ── 初始化 ────────────────────────────────────────────────

    def _on_init_done(self):
        """第一个电机完成采集后调用，创建轨迹 + 激活增益"""
        self.init_done = True
        now = self.get_clock().now().nanoseconds * 1e-9

        # 激活增益
        for i in range(TOTAL_MOTORS):
            self.kp[i] = self._kp_configured
            self.kd[i] = self._kd_configured

        # 创建轨迹
        for i in range(TOTAL_MOTORS):
            if self.q_ini[i] is not None:
                q_start = self.q_ini[i]
                q_end = q_start + self.target_offsets[i]
                self.trajectories[i] = QuinticTrajectory(
                    q_start, q_end, self.traj_duration)
                self.get_logger().info(
                    f'  M{i:02d} traj: {q_start:.1f} -> {q_end:.1f} '
                    f'({self.traj_duration:.1f}s)')
                # 立即把 cmd 设为初始位置，下一轮 sendRecv 时电机不会跳
                self._set_cmd(i, q_start, 0.0)

        self.traj_start_time = now
        active = sum(1 for q in self.q_ini if q is not None)
        self.get_logger().info(
            f'Init complete: {active}/{TOTAL_MOTORS} motors')

    # ── 端口健康 ──────────────────────────────────────────────

    def _skip_port(self, port):
        if not port['suspended']:
            return False
        idle = self._cycle_count - port.get('probe_cycle', 0)
        if idle >= self._probe_every:
            port['suspended'] = False
            return False
        return True

    def _update_health(self, port, ok):
        if ok:
            port['failures'] = 0
            return
        port['failures'] += 1
        if port['failures'] >= self._suspend_after:
            if not port['suspended']:
                self.get_logger().warn(
                    f'  {port["path"]} suspended after {port["failures"]} failures')
            port['suspended'] = True
            port['probe_cycle'] = self._cycle_count

    # ── 辅助 ──────────────────────────────────────────────────

    @contextmanager
    def _quiet_sdk(self):
        old_out = os.dup(1)
        old_err = os.dup(2)
        os.dup2(self._devnull, 1)
        os.dup2(self._devnull, 2)
        try:
            yield
        finally:
            os.dup2(old_out, 1)
            os.dup2(old_err, 2)
            os.close(old_out)
            os.close(old_err)

    @staticmethod
    def _empty_state():
        state = MotorState()
        state.tau = [0.0] * TOTAL_MOTORS
        state.q = [0.0] * TOTAL_MOTORS
        state.dq = [0.0] * TOTAL_MOTORS
        state.temp = [0.0] * TOTAL_MOTORS
        state.merror = [-1] * TOTAL_MOTORS
        return state

    # ── 显示 ──────────────────────────────────────────────────

    def _update_display(self, state):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_display < 0.5:
            return
        self._last_display = now

        import shutil
        term_w = shutil.get_terminal_size((120, 24)).columns
        ratio = self.gear_ratio
        lines = []

        active = [i for i in range(TOTAL_MOTORS) if self.seen[i]]
        for i in active:
            port = self.ports[i // MOTORS_PER_PORT]
            flag = 'S' if port['suspended'] else ' '
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
            lines.append(
                f"{flag}M{i:02d} "
                f"real={state.q[i]/ratio:+7.3f} "
                f"dq={state.dq[i]/ratio:+7.3f} "
                f"tau={state.tau[i]:+6.2f} "
                f"T={state.temp[i]:4.0f} "
                f"err={state.merror[i]:2d} [{prog:>8}]"
            )
            lines[-1] = lines[-1][:term_w - 1]

        alive = sum(1 for p in self.ports if p['active'] and not p['suspended'])
        suspended = sum(1 for p in self.ports if p['active'] and p['suspended'])
        dead = sum(1 for p in self.ports if not p['active'])
        parts = []
        if alive:     parts.append(f'{alive} OK')
        if suspended: parts.append(f'{suspended} suspended')
        if dead:      parts.append(f'{dead} dead')
        lines.append(f'--- {", ".join(parts) or "no ports"} ---')

        if not self._display_ready:
            print('\n' * max(len(lines) - 1, 0))
            self._display_ready = True

        print(f"\033[{len(lines)}A", end='')
        for line in lines:
            print(f"\033[K{line}")
        print('\033[J', end='', flush=True)

    # ── 停机 ──────────────────────────────────────────────────

    def __del__(self):
        ports = getattr(self, 'ports', [])
        mode = getattr(self, 'motor_mode', 0)
        for port in ports:
            if not port['active']:
                continue
            for local_id in range(MOTORS_PER_PORT):
                try:
                    port['cmd'].id = local_id
                    port['cmd'].mode = mode
                    port['cmd'].q = 0.0
                    port['cmd'].kp = 0.0
                    port['cmd'].kd = 0.0
                    port['cmd'].dq = 0.0
                    port['cmd'].tau = 0.0
                    port['serial'].sendRecv(port['cmd'], port['data'])
                except Exception:
                    break
        self.get_logger().info('All motors stopped')


def main(args=None):
    rclpy.init(args=args)
    node = MotorRotatesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
