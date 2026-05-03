import os
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


def _suppress_fd(fd_target, null_fd):
    """Redirect fd_target to /dev/null, return old fd that must be restored."""
    old = os.dup(fd_target)
    os.dup2(null_fd, fd_target)
    return old


def _restore_fd(fd_target, old):
    os.dup2(old, fd_target)
    os.close(old)


class MotorRotatesNode(Node):
    def __init__(self):
        super().__init__('motor_rotates')
        self.get_logger().info('MotorRotatesNode starting')

        self.motor_mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)

        self.ports = []
        for cfg in PORTS:
            port = {
                'path': cfg['path'],
                'base': cfg['base'],
                'serial': None,
                'cmd': None,
                'data': None,
                'active': False,
                'failures': 0,
                'suspended': False,
                'probe_cycle': 0,
            }
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
            self.get_logger().error(
                'No serial ports available — motor driver is idle')

        self.kp = [0.0] * TOTAL_MOTORS
        self.kd = [0.0] * TOTAL_MOTORS
        self.seen = [False] * TOTAL_MOTORS
        self.cmd_q = [0.0] * TOTAL_MOTORS
        self._display_ready = False
        self._suspend_threshold = 10       # 端口连续失败N次后暂停
        self._probe_interval_cycles = 100  # 暂停后每N个周期探测一次
        self._cycle_count = 0
        self._devnull = os.open(os.devnull, os.O_WRONLY)
        self._last_display_time = 0.0
        self._display_interval = 0.5

        self.cmd_sub = self.create_subscription(
            MotorCommand, 'motor_command', self.command_callback, 10)
        self.pd_sub = self.create_subscription(
            MotorPd, 'motor_pd', self.pd_callback, 10)
        self.state_pub = self.create_publisher(
            MotorState, 'motor_state', 10)

    def pd_callback(self, msg):
        self.kp = list(msg.kp)
        self.kd = list(msg.kd)

    _debug_attrs_logged = False

    @classmethod
    def _is_valid_data(cls, data):
        # 一次性调试：打印 MotorData 所有属性，确认 correct 字段是否可用
        if not cls._debug_attrs_logged:
            cls._debug_attrs_logged = True
            attrs = [a for a in dir(data) if not a.startswith('_')]
            has_correct = hasattr(data, 'correct')
            correct_val = getattr(data, 'correct', 'N/A')
            print(f'\n[MotorData attrs] correct={correct_val}, hasattr={has_correct}')
            print(f'[MotorData attrs] public fields: {attrs}')

        # 1. SDK CRC check — 硬件级校验，最可靠
        if hasattr(data, 'correct') and not data.correct:
            return False

        # 2. Type check — 字段必须存在且类型正确
        for attr in ('tau', 'q', 'dq', 'temp', 'merror'):
            val = getattr(data, attr, None)
            if val is None:
                return False
            if attr == 'merror':
                if not isinstance(val, int):
                    return False
            elif not isinstance(val, (int, float)):
                return False

        # 3. All-zero rejection — RS485通信失败时sendRecv返回全零但无异常
        if float(data.q) == 0.0 and float(data.dq) == 0.0 \
           and float(data.tau) == 0.0 and float(data.temp) == 0:
            return False

        # 4. Temp range — 正常电机温度 -40~150°C，异常值如 65535(0xFFFF) 说明通信失败
        temp = float(data.temp)
        if temp < -40.0 or temp > 150.0:
            return False

        # 5. Merror range — GO-M8010-6 错误码范围 0~7，超出说明数据损坏
        merror = int(data.merror)
        if merror < 0 or merror > 255:
            return False

        return True

    def _fill_default_state(self):
        state = MotorState()
        state.tau = [0.0] * TOTAL_MOTORS
        state.q = [0.0] * TOTAL_MOTORS
        state.dq = [0.0] * TOTAL_MOTORS
        state.temp = [0.0] * TOTAL_MOTORS
        state.merror = [-1] * TOTAL_MOTORS
        return state

    def command_callback(self, msg):
        state = self._fill_default_state()
        self._cycle_count += 1

        for i in range(TOTAL_MOTORS):
            self.cmd_q[i] = msg.q[i]

        for port in self.ports:
            if not port['active']:
                continue

            # 端口暂停退避：死端口只偶尔探测
            if port['suspended']:
                if self._cycle_count - port.get('probe_cycle', 0) < self._probe_interval_cycles:
                    # 保持 merror=-1，不通信
                    continue
                # 探测周期到了，临时解除一次
                port['suspended'] = False

            port_ok_count = 0
            for local_id in range(MOTORS_PER_PORT):
                gi = port['base'] + local_id
                port['cmd'].id = msg.id[gi]
                port['cmd'].mode = self.motor_mode
                port['cmd'].q = msg.q[gi]
                port['cmd'].kp = self.kp[gi]
                port['cmd'].kd = self.kd[gi]
                port['cmd'].dq = msg.dq[gi]
                port['cmd'].tau = msg.tau[gi]
                try:
                    # 屏蔽 C++ SDK 的 printf 警告
                    old_out = _suppress_fd(1, self._devnull)
                    old_err = _suppress_fd(2, self._devnull)
                    try:
                        ok = port['serial'].sendRecv(port['cmd'], port['data'])
                    finally:
                        _restore_fd(1, old_out)
                        _restore_fd(2, old_err)

                    if ok and self._is_valid_data(port['data']):
                        state.tau[gi] = float(port['data'].tau)
                        state.q[gi] = float(port['data'].q)
                        state.dq[gi] = float(port['data'].dq)
                        state.temp[gi] = float(port['data'].temp)
                        state.merror[gi] = int(port['data'].merror)
                        self.seen[gi] = True
                        port_ok_count += 1
                except Exception:
                    pass

            # 只有端口上全部电机都失败才计入端口失败
            if port_ok_count > 0:
                port['failures'] = 0
            else:
                port['failures'] += 1
                if port['failures'] >= self._suspend_threshold:
                    if not port['suspended']:
                        self.get_logger().warn(
                            f'  {port["path"]} suspended after '
                            f'{port["failures"]} failures')
                    port['suspended'] = True
                    port['probe_cycle'] = self._cycle_count

        self.state_pub.publish(state)
        self._update_display(state)

    def _update_display(self, state):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_display_time < self._display_interval:
            return
        self._last_display_time = now

        import shutil
        term_w = shutil.get_terminal_size((120, 24)).columns

        active_indices = [i for i in range(TOTAL_MOTORS) if self.seen[i]]
        ratio = queryGearRatio(MotorType.GO_M8010_6)
        lines = []

        for i in active_indices:
            port_idx = i // MOTORS_PER_PORT
            port = self.ports[port_idx]
            port_ok = port['active'] and not port['suspended']
            if not port_ok:
                flag = 'S'  # suspended
            elif state.merror[i] != -1:
                flag = ' '
            else:
                flag = '?'
            line = (f"{flag}M{i:02d} "
                    f"cmd={self.cmd_q[i]/ratio:+7.3f} "
                    f"real={state.q[i]/ratio:+7.3f} "
                    f"dq={state.dq[i]/ratio:+7.3f} "
                    f"tau={state.tau[i]:+6.2f} "
                    f"err={state.merror[i]:2d}")
            lines.append(line[:term_w - 1])

        # 汇总行：活跃端口 / 死端口
        alive = sum(1 for p in self.ports if p['active'] and not p['suspended'])
        dead = sum(1 for p in self.ports if p['active'] and p['suspended'])
        inactive = sum(1 for p in self.ports if not p['active'])
        parts = []
        if alive:
            parts.append(f'{alive} port(s) OK')
        if dead:
            parts.append(f'{dead} suspended')
        if inactive:
            parts.append(f'{inactive} dead')
        if not parts:
            parts.append('no ports')
        lines.append(f'--- {", ".join(parts)} ---')

        if not active_indices and not lines:
            return

        if not self._display_ready:
            print('\n' * (len(lines) - 1))
            self._display_ready = True

        print(f"\033[{len(lines)}A", end='')
        for line in lines:
            print(f"\033[K{line}")
        print('\033[J', end='', flush=True)

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
