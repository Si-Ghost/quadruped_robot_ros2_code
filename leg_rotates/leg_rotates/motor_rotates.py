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

        self.ports = []
        for cfg in PORTS:
            port = {
                'path': cfg['path'],
                'base': cfg['base'],
                'serial': None,
                'cmd': None,
                'data': None,
                'active': False,
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

        self.cmd_sub = self.create_subscription(
            MotorCommand, 'motor_command', self.command_callback, 10)
        self.pd_sub = self.create_subscription(
            MotorPd, 'motor_pd', self.pd_callback, 10)
        self.state_pub = self.create_publisher(
            MotorState, 'motor_state', 10)

    def pd_callback(self, msg):
        self.kp = list(msg.kp)
        self.kd = list(msg.kd)

    @staticmethod
    def _is_valid_data(data):
        for attr in ('tau', 'q', 'dq', 'temp', 'merror'):
            val = getattr(data, attr, None)
            if val is None:
                return False
            if attr == 'merror':
                if not isinstance(val, int):
                    return False
            elif not isinstance(val, (int, float)):
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

        for i in range(TOTAL_MOTORS):
            self.cmd_q[i] = msg.q[i]

        for port in self.ports:
            if not port['active']:
                continue
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
                    port['serial'].sendRecv(port['cmd'], port['data'])
                    if self._is_valid_data(port['data']):
                        state.tau[gi] = float(port['data'].tau)
                        state.q[gi] = float(port['data'].q)
                        state.dq[gi] = float(port['data'].dq)
                        state.temp[gi] = float(port['data'].temp)
                        state.merror[gi] = int(port['data'].merror)
                        self.seen[gi] = True
                except Exception:
                    pass

        self.state_pub.publish(state)
        self._update_display(state)

    def _update_display(self, state):
        import shutil
        term_w = shutil.get_terminal_size((120, 24)).columns

        active_indices = [i for i in range(TOTAL_MOTORS) if self.seen[i]]
        if not active_indices:
            return

        ratio = queryGearRatio(MotorType.GO_M8010_6)
        lines = []
        for i in active_indices:
            port_idx = i // MOTORS_PER_PORT
            port_ok = self.ports[port_idx]['active']
            flag = ' ' if (port_ok and state.merror[i] != -1) else '*'
            line = (f"{flag}M{i:02d} "
                    f"cmd={self.cmd_q[i]/ratio:+7.3f} "
                    f"real={state.q[i]/ratio:+7.3f} "
                    f"dq={state.dq[i]/ratio:+7.3f} "
                    f"tau={state.tau[i]:+6.2f} "
                    f"err={state.merror[i]:2d}")
            lines.append(line[:term_w - 1])

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
