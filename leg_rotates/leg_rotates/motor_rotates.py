import rclpy
from rclpy.node import Node
import time

# 从闭源 SDK 导入所有内容（不要手动添加 sys.path）
from unitree_actuator_sdk import *

class MotorRotatesNode(Node):
    def __init__(self):
        super().__init__('motor_rotates')
        self.get_logger().info('MotorRotatesNode started')

        # 初始化串口（根据实际设备路径调整）
        self.serial = SerialPort('/dev/ttyUSB0')
        self.cmd = MotorCmd()
        self.data = MotorData()

        # 设置电机类型
        self.cmd.motorType = MotorType.GO_M8010_6
        self.data.motorType = MotorType.GO_M8010_6

        # 计算目标位置（-10° 转换为弧度，并考虑减速比）
        self.target_q = -10 * (3.14159 / 180.0) * queryGearRatio(MotorType.GO_M8010_6)
        self.q_ini = self.read_initial_position(2)

        # 创建一个定时器，以 200Hz 的频率运行控制循环
        self.timer = self.create_timer(0.005, self.control_loop)  # 5ms = 200Hz

    def read_initial_position(self, motor_id):
        """读取电机初始位置"""
        self.cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        self.cmd.id = motor_id
        self.cmd.q = 0
        self.cmd.dq = 0
        self.cmd.kp = 0
        self.cmd.kd = 0
        self.cmd.tau = 0
        self.serial.sendRecv(self.cmd, self.data)
        return self.data.q

    def update_display(self, data):
        """更新终端显示"""
        lines = [
            f"力矩: {data.tau}",
            f"电流: {data.tau/0.63895}",
            f"电机角度: {data.q/queryGearRatio(MotorType.GO_M8010_6)}",
            f"电机角速度: {data.dq/queryGearRatio(MotorType.GO_M8010_6)}",
            f"温度: {data.temp}",
            f"错误: {data.merror}"
        ]
        output = "\n".join(lines)
        print(output)
        print(f"\033[{len(lines)}F", end="", flush=True)

    def control_loop(self):
        """控制循环，由定时器调用"""
        self.cmd.id = 0
        self.cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        self.cmd.q = self.target_q + self.q_ini
        self.cmd.kp = 0.4
        self.cmd.kd = 0.01
        self.cmd.dq = 0
        self.cmd.tau = 0

        self.serial.sendRecv(self.cmd, self.data)
        self.update_display(self.data)

    def __del__(self):
        """析构时发送零力矩指令，防止电机卡死"""
        self.cmd.kp = 0
        self.cmd.kd = 0
        self.cmd.tau = 0
        self.serial.sendRecv(self.cmd, self.data)
        self.get_logger().info("已停止控制")

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