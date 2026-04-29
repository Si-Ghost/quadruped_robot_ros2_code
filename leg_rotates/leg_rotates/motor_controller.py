import rclpy
from rclpy.node import Node
from interface_type.msg import MotorCommand, MotorState, MotorPd

from unitree_actuator_sdk import *

TOTAL_MOTORS = 12
INIT_TIMEOUT_ITER = 200  # ~1 s at 200 Hz


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info('MotorControllerNode started')

        self.cmd_pub = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.pd_pub = self.create_publisher(MotorPd, 'motor_pd', 10)
        self.state_sub = self.create_subscription(
            MotorState, 'motor_state', self.state_callback, 10)

        self.gear_ratio = queryGearRatio(MotorType.GO_M8010_6)
        self.target_offset = -10.0 * (3.14159 / 180.0) * self.gear_ratio

        self.q_ini = [None] * TOTAL_MOTORS
        self.init_counter = 0
        self.init_done = False

        self.publish_pd()
        self.pd_timer = self.create_timer(1.0, self.publish_pd)
        self.control_timer = self.create_timer(0.005, self.control_loop)

    def publish_pd(self):
        pd_msg = MotorPd()
        pd_msg.kp = [0.4] * TOTAL_MOTORS
        pd_msg.kd = [0.01] * TOTAL_MOTORS
        self.pd_pub.publish(pd_msg)

    def state_callback(self, msg):
        if not self.init_done:
            for i in range(TOTAL_MOTORS):
                if msg.merror[i] != -1 and self.q_ini[i] is None:
                    self.q_ini[i] = msg.q[i]
                    self.get_logger().info(f'  Motor {i} init q={self.q_ini[i]:.3f}')

        self.update_display(msg)

    def update_display(self, msg):
        ratio = self.gear_ratio
        lines = []
        for i in range(TOTAL_MOTORS):
            status = '*' if msg.merror[i] == -1 else ' '
            lines.append(
                f"{status}M{i:02d} tau={msg.tau[i]:+6.2f} q={msg.q[i]/ratio:+7.3f} "
                f"dq={msg.dq[i]/ratio:+7.3f} T={msg.temp[i]:4.0f} err={msg.merror[i]:2d}"
            )
        output = "\n".join(lines)
        print(output)
        print(f"\033[{len(lines)}F", end="", flush=True)

    def control_loop(self):
        if not self.init_done:
            self.init_counter += 1
            all_ready = all(q is not None for q in self.q_ini)
            timed_out = self.init_counter >= INIT_TIMEOUT_ITER

            if all_ready or timed_out:
                self.init_done = True
                active = sum(1 for q in self.q_ini if q is not None)
                self.get_logger().info(
                    f'Init complete: {active}/{TOTAL_MOTORS} motors active'
                )

        cmd = MotorCommand()
        cmd.id = [i % 3 for i in range(TOTAL_MOTORS)]
        cmd.q = [0.0] * TOTAL_MOTORS
        cmd.dq = [0.0] * TOTAL_MOTORS
        cmd.tau = [0.0] * TOTAL_MOTORS

        for i in range(TOTAL_MOTORS):
            if self.q_ini[i] is not None:
                cmd.q[i] = self.target_offset + self.q_ini[i]
            else:
                cmd.q[i] = 0.0

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
