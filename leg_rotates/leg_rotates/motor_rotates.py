import os
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from interface_type.msg import MotorCommand, MotorState, MotorPd

from unitree_actuator_sdk import *

PORTS = [
    {'path': '/dev/ttyUSB0', 'base': 0},
    {'path': '/dev/ttyUSB1', 'base': 3},
    {'path': '/dev/ttyUSB2', 'base': 6},
    {'path': '/dev/ttyUSB3', 'base': 9},
]
MOTORS_PER_PORT = 3
TOTAL_MOTORS = 12


class MotorRotatesNode(Node):
    def __init__(self):
        super().__init__('motor_rotates')
        self.get_logger().info('MotorRotatesNode starting')

        self.motor_mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        self._devnull = os.open(os.devnull, os.O_WRONLY)

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

        self.kp = [0.0] * TOTAL_MOTORS
        self.kd = [0.0] * TOTAL_MOTORS
        self.seen = [False] * TOTAL_MOTORS
        self.cmd_q = [0.0] * TOTAL_MOTORS

        self._cycle_count = 0
        self._suspend_after = 10
        self._probe_every = 100

        self._display_ready = False
        self._last_display = 0.0

        self.cmd_sub = self.create_subscription(
            MotorCommand, 'motor_command', self.command_callback, 10)
        self.pd_sub = self.create_subscription(
            MotorPd, 'motor_pd', self.pd_callback, 10)
        self.state_pub = self.create_publisher(
            MotorState, 'motor_state', 10)

    # ── ROS callbacks ──────────────────────────────────────────

    def pd_callback(self, msg):
        self.kp = list(msg.kp)
        self.kd = list(msg.kd)

    def command_callback(self, msg):
        state = self._empty_state()
        self._cycle_count += 1
        for i in range(TOTAL_MOTORS):
            self.cmd_q[i] = msg.q[i]

        for port in self.ports:
            if not port['active']:
                continue
            if self._skip_port(port):
                continue

            alive = 0
            for local_id in range(MOTORS_PER_PORT):
                gi = port['base'] + local_id
                r = self._read_motor(port, local_id,
                                     q=msg.q[gi], dq=msg.dq[gi],
                                     tau=msg.tau[gi], kp=self.kp[gi],
                                     kd=self.kd[gi], motor_id=msg.id[gi])
                if r is not None:
                    state.tau[gi] = r['tau']
                    state.q[gi] = r['q']
                    state.dq[gi] = r['dq']
                    state.temp[gi] = r['temp']
                    state.merror[gi] = r['merror']
                    self.seen[gi] = True
                    alive += 1

            self._update_health(port, alive > 0)

        self.state_pub.publish(state)
        self._update_display(state)

    # ── motor I/O ──────────────────────────────────────────────

    def _read_motor(self, port, local_id, *, q, dq, tau, kp, kd, motor_id):
        """读取一个电机。成功返回 dict，失败返回 None。"""
        port['cmd'].id = motor_id
        port['cmd'].mode = self.motor_mode
        port['cmd'].q = q
        port['cmd'].dq = dq
        port['cmd'].tau = tau
        port['cmd'].kp = kp
        port['cmd'].kd = kd

        with self._quiet_sdk():
            ok = port['serial'].sendRecv(port['cmd'], port['data'])

        if ok and port['data'].correct:
            d = port['data']
            return {'q': float(d.q), 'dq': float(d.dq), 'tau': float(d.tau),
                    'temp': float(d.temp), 'merror': int(d.merror)}
        return None

    # ── port health ────────────────────────────────────────────

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

    # ── helpers ────────────────────────────────────────────────

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

    # ── display ────────────────────────────────────────────────

    def _update_display(self, state):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_display < 0.5:
            return
        self._last_display = now

        import shutil
        term_w = shutil.get_terminal_size((120, 24)).columns
        ratio = queryGearRatio(MotorType.GO_M8010_6)
        lines = []

        active = [i for i in range(TOTAL_MOTORS) if self.seen[i]]
        for i in active:
            port = self.ports[i // MOTORS_PER_PORT]
            flag = 'S' if port['suspended'] else ' '
            lines.append(
                f"{flag}M{i:02d} "
                f"cmd={self.cmd_q[i]/ratio:+7.3f} "
                f"real={state.q[i]/ratio:+7.3f} "
                f"dq={state.dq[i]/ratio:+7.3f} "
                f"tau={state.tau[i]:+6.2f} "
                f"err={state.merror[i]:2d}"
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

    # ── shutdown ───────────────────────────────────────────────

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
