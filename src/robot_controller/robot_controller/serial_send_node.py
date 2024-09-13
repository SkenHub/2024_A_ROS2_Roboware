import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import serial
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')

        # 各トピックからデータを購読
        self.websocket_subscription = self.create_subscription(
            String,
            'web_socket_pub',
            self.websocket_callback,
            10
        )
        self.controller_subscription = self.create_subscription(
            Float32MultiArray,
            'cmd_vel',
            self.controller_callback,
            10
        )

        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # 実際のシリアルポートに変更

        # 初期値を設定
        self.command_number = 0
        self.mode = 0
        self.Vx = 0.0
        self.Vy = 0.0
        self.omega = 0.0

    def websocket_callback(self, msg):
        # WebSocketからのデータをパース
        data = msg.data.split(',')
        if len(data) >= 2:
            self.command_number = int(data[0])  # ${behavior}
            self.mode = int(data[1])  # ${mode}
        else:
            self.get_logger().error('Received data from WebSocket does not have enough values for command_number and mode.')

    def controller_callback(self, msg):
        # Controller nodeからのデータを取得
        if len(msg.data) >= 3:
            self.Vx = float(msg.data[0])
            self.Vy = float(msg.data[1])
            self.omega = float(msg.data[2])

            # データを送信
            self.get_logger().info(f"Sending data: Command Number={self.command_number}, Mode={self.mode}, Vx={self.Vx}, Vy={self.Vy}, Omega={self.omega}")
            self.send_data_to_mcu(self.command_number, self.mode, self.Vx, self.Vy, self.omega)
        else:
            self.get_logger().error('Received data from Controller node does not have enough values for Vx, Vy, and omega.')

    def send_data_to_mcu(self, command_number, mode, Vx, Vy, omega):
        HEADER = b'\xA5\xA5'
        Vx = max(-32768, min(32767, int(Vx)))
        Vy = max(-32768, min(32767, int(Vy)))
        omega = max(-32768, min(32767, int(omega)))
        data = struct.pack('>BBhhh', command_number, mode, Vx, Vy, omega)
        packet = HEADER + data
        self.serial_port.write(packet)

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
