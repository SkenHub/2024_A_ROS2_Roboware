import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class CmdVelToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')

        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.seq = 0  # 初期シーケンス番号

    def listener_callback(self, msg):
        if len(msg.data) == 5:
            speed, direction, theta, team_color, action_number = msg.data
            # データをパック
            send_data = struct.pack('>BBfffBB', 0xA5, 0xA5, speed, direction, theta, int(team_color), int(action_number))
            # シーケンス番号をインクリメントし、255を超えたら0にリセット
            self.seq = (self.seq + 1) % 256
            # チェックサムを計算（0xA5とシーケンス番号を除く）
            checksum = sum(send_data[2:]) % 256
            # シーケンス番号を追加
            send_data += struct.pack('B', self.seq)
            # チェックサムをデータに追加
            send_data += struct.pack('B', checksum)
            self.ser.write(send_data)
            self.get_logger().info(f"Sent data: {msg.data}")
            self.get_logger().info(f"Sent packed data: {send_data.hex()}")

            

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
