import rclpy
from rclpy.node import Node
from interface_type.msg import MotorCommand, MotorState

from unitree_actuator_sdk import *


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info('MotorControllerNode started')

        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.state_sub = self.create_subscription(
            MotorState,
            'motor_state',
            self.state_callback,
            10)

        self.gear_ratio = queryGearRatio(MotorType.GO_M8010_6)
        self.target_q = -10.0 * (3.14159 / 180.0) * self.gear_ratio
        self.q_ini = None
        self.init_done = False

        self.request_initial_position()
        self.timer = self.create_timer(0.005, self.control_loop)

    def request_initial_position(self):
        cmd = MotorCommand()
        cmd.id = 2
        cmd.q = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0
        cmd.dq = 0.0
        cmd.tau = 0.0
        self.publisher.publish(cmd)

    def state_callback(self, msg):
        if not self.init_done:
            self.q_ini = msg.q
            self.init_done = True
            self.get_logger().info(f'Initial position: {self.q_ini:.3f}')

        ratio = self.gear_ratio
        lines = [
            f"力矩: {msg.tau:.3f}",
            f"电机角度: {msg.q / ratio:.3f}",
            f"电机角速度: {msg.dq / ratio:.3f}",
            f"温度: {msg.temp:.1f}",
            f"错误: {msg.merror}"
        ]
        output = "\n".join(lines)
        print(output)
        print(f"\033[{len(lines)}F", end="", flush=True)

    def control_loop(self):
        if not self.init_done:
            self.request_initial_position()
            return

        cmd = MotorCommand()
        cmd.id = 0
        cmd.q = self.target_q + self.q_ini
        cmd.kp = 0.4
        cmd.kd = 0.01
        cmd.dq = 0.0
        cmd.tau = 0.0
        self.publisher.publish(cmd)

    def __del__(self):
        cmd = MotorCommand()
        cmd.id = 0
        cmd.kp = 0.0
        cmd.kd = 0.0
        cmd.tau = 0.0
        self.publisher.publish(cmd)
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
