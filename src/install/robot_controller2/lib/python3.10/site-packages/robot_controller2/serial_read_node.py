import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialReadNode(Node):
    def __init__(self):
        super().__init__('serial_read_node')

        # シリアルポートの設定
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.buffer = b''

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)

        # ヘッダーを探してデータを解析
        while len(self.buffer) >= 10:  # ヘッダー2バイト + データ8バイト = 10バイト
            start = self.buffer.find(b'\xA5\xA5')
            if start != -1 and len(self.buffer[start:]) >= 10:
                data = self.buffer[start+2:start+10]
                self.buffer = self.buffer[start+10:]

                if len(data) == 8:  # 期待されるデータ長を確認
                    try:
                        action_number, mode, X, Y, theta = struct.unpack('>BBhhh', data)
                        position_msg = Float32MultiArray()
                        position_msg.data = [
                            float(X),  # x位置
                            float(Y),  # y位置
                            float(theta),  # ロボットの角度
                            float(mode),  # モード
                            float(action_number)  # 動作番号
                        ]
                        self.publisher_.publish(position_msg)
                        self.get_logger().info(f"Published position data: {position_msg.data}")
                    except struct.error as e:
                        self.get_logger().error(f"Unpacking error: {e}")
            else:
                # データ不足の場合は読み込みを続ける
                self.buffer = self.buffer[start+1:] if start != -1 else self.buffer[1:]

def main(args=None):
    rclpy.init(args=args)
    node = SerialReadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
