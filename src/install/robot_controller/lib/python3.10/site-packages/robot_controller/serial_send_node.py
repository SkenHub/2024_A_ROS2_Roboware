import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')

        # シリアルポートの設定
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if len(msg.data) == 5:
            command_number = int(msg.data[0])  # 指示番号
            mode = int(msg.data[1])  # モード
            Vx = int(msg.data[2])  # x方向の速度 [mm/s]
            Vy = int(msg.data[3])  # y方向の速度 [mm/s]
            omega = int(msg.data[4])  # 角速度 [deg/s]

            # シリアルデータのパッキング
            data = struct.pack('>BBhhh', command_number, mode, Vx, Vy, omega)
            packet = b'\xA5\xA5' + data  # ヘッダー + データ
            self.ser.write(packet)  # シリアルポートに送信
            self.get_logger().info(f"Sent data: {msg.data}")
            self.get_logger().info(f"Sent packed data: {packet.hex()}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
