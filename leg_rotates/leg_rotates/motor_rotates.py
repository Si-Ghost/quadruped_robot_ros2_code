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

    def command_callback(self, msg):
        self.cmd.id = msg.id
        self.cmd.mode = self.motor_mode
        self.cmd.q = msg.q
        self.cmd.kp = msg.kp
        self.cmd.kd = msg.kd
        self.cmd.dq = msg.dq
        self.cmd.tau = msg.tau

        self.serial.sendRecv(self.cmd, self.data)

        state_msg = MotorState()
        state_msg.tau = self.data.tau
        state_msg.q = self.data.q
        state_msg.dq = self.data.dq
        state_msg.temp = self.data.temp
        state_msg.merror = self.data.merror
        self.publisher.publish(state_msg)

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
        self.serial.sendRecv(self.cmd, self.data)
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
