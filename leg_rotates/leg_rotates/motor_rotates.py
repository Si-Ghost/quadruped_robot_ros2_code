import rclpy
from rclpy.node import Node
from interface_type.msg import MotorCommand, MotorState

from unitree_actuator_sdk import *


class MotorRotatesNode(Node):
    def __init__(self):
        super().__init__('motor_rotates')
        self.get_logger().info('MotorRotatesNode started')

        self.serial = SerialPort('/dev/ttyUSB0')
        self.cmd = MotorCmd()
        self.data = MotorData()

        self.cmd.motorType = MotorType.GO_M8010_6
        self.data.motorType = MotorType.GO_M8010_6
        self.motor_mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)

        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.command_callback,
            10)
        self.publisher = self.create_publisher(MotorState, 'motor_state', 10)

    def _is_valid_data(self):
        """Check if motor response data has valid numeric fields."""
        for attr in ('tau', 'q', 'dq', 'temp', 'merror'):
            val = getattr(self.data, attr, None)
            if val is None:
                return False
            if attr == 'merror':
                if not isinstance(val, int):
                    return False
            else:
                if not isinstance(val, (int, float)):
                    return False
        return True

    def _build_state(self, comm_ok):
        state = MotorState()
        if comm_ok:
            state.tau = float(self.data.tau)
            state.q = float(self.data.q)
            state.dq = float(self.data.dq)
            state.temp = float(self.data.temp)
            state.merror = int(self.data.merror)
        else:
            state.tau = 0.0
            state.q = 0.0
            state.dq = 0.0
            state.temp = 0.0
            state.merror = -1
        return state

    def command_callback(self, msg):
        self.cmd.id = msg.id
        self.cmd.mode = self.motor_mode
        self.cmd.q = msg.q
        self.cmd.kp = msg.kp
        self.cmd.kd = msg.kd
        self.cmd.dq = msg.dq
        self.cmd.tau = msg.tau

        comm_ok = False
        try:
            self.serial.sendRecv(self.cmd, self.data)
            comm_ok = self._is_valid_data()
        except Exception:
            self.get_logger().error('Motor communication failed', throttle_duration_sec=1.0)

        state_msg = self._build_state(comm_ok)
        self.publisher.publish(state_msg)

        if comm_ok:
            self.update_display(self.data)

    def update_display(self, data):
        ratio = queryGearRatio(MotorType.GO_M8010_6)
        lines = [
            f"力矩: {data.tau:.3f}",
            f"电流: {data.tau / 0.63895:.3f}",
            f"电机角度: {data.q / ratio:.3f}",
            f"电机角速度: {data.dq / ratio:.3f}",
            f"温度: {data.temp:.1f}",
            f"错误: {data.merror}"
        ]
        output = "\n".join(lines)
        print(output)
        print(f"\033[{len(lines)}F", end="", flush=True)

    def __del__(self):
        self.cmd.kp = 0.0
        self.cmd.kd = 0.0
        self.cmd.tau = 0.0
        try:
            self.serial.sendRecv(self.cmd, self.data)
        except Exception:
            pass
        self.get_logger().info('Motor stopped')


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
