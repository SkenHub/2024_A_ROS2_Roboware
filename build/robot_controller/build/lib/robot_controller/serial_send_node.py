import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'cmd_vel', self.send_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # 実際のシリアルポートに変更

    def send_callback(self, msg):
        # データが5つ以上の場合、最初の3つのデータ（Vx, Vy, omega）を取得
        if len(msg.data) >= 3:
            Vx, Vy, omega = msg.data[:3]
            command_number = 1  # デフォルトのコマンド番号
            mode = 1  # デフォルトのモード
            self.get_logger().info(f"Sending data: Vx={Vx}, Vy={Vy}, Omega={omega}")
            self.send_data_to_mcu(command_number, mode, Vx, Vy, omega)
        else:
            self.get_logger().error('Received data does not have enough values for Vx, Vy, and omega.')

    def send_data_to_mcu(self, command_number, mode, Vx, Vy, omega):
        HEADER = b'\xA5\xA5'
        Vx = max(-32768, min(32767, int(Vx)))
        Vy = max(-32768, min(32767, int(Vy)))
        omega = max(-32768, min(32767, int(omega)))
        data = struct.pack('>BBhhh', command_number, mode, Vx, Vy, omega)
        packet = HEADER + data
        self.serial_port.write(packet)
        self.get_logger().info(f"Sent packet: {packet.hex()}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
