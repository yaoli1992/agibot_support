#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import os

class JointStateListener(Node):

    def __init__(self):
        super().__init__('joint_state_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # 打开文件以保存关节状态数据
        self.file_path = os.path.join(os.getcwd(), 'joint_states.txt')
        self.file = open(self.file_path, 'w')

    def listener_callback(self, msg):
        # 获取关节状态数据
        joint_positions = msg.position
        joint_velocities = msg.velocity
        joint_efforts = msg.effort
        data = []
        data+=[0,0,0]
        data+=joint_positions[15:20]
        data+=joint_positions[23:28]
        # 合并关节位置、速度、扭矩（或力）的数据为一行，并用逗号隔开
        data_row = " ".join(map(str, data)) + "\n"

        # 将数据行写入文件
        self.file.write(data_row)

    def __del__(self):
        # 程序结束时关闭文件
        self.file.close()

def main(args=None):
    rclpy.init(args=args)

    joint_state_listener = JointStateListener()

    try:
        rclpy.spin(joint_state_listener)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

