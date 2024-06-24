import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialToPositionNode(Node):
    def __init__(self):
        super().__init__('serial_to_position_node')

        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # より頻繁にバッファをチェックするためのタイマー
        self.buffer = b''

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)

        # バッファに完全なパケット（18バイト、16バイトのデータ + 1バイトのシーケンス番号 + 1バイトのチェックサム）が含まれているか確認
        while len(self.buffer) >= 18:
            # パケットの開始を見つける
            start = self.buffer.find(b'\xA5\xA5')
            if start != -1 and len(self.buffer[start:]) >= 18:
                data = self.buffer[start+2:start+16]
                seq = self.buffer[start+16]
                received_checksum = self.buffer[start+17]
                self.buffer = self.buffer[start+18:]
                calculated_checksum = sum(data) % 256

                if calculated_checksum == received_checksum:
                    try:
                        position_data = struct.unpack('fffBB', data)
                        position_msg = Float32MultiArray()
                        position_msg.data = [
                            float(position_data[0]),  # x
                            float(position_data[1]),  # y
                            float(position_data[2]),  # theta
                            float(position_data[3]),  # team color
                            float(position_data[4])   # action number
                        ]
                        self.publisher_.publish(position_msg)
                        self.get_logger().info(f"Published position data: {position_msg.data}")
                    except struct.error as e:
                        self.get_logger().error(f"Unpacking error: {e}")
                else:
                    self.get_logger().error(f"Checksum error: received {received_checksum}, calculated {calculated_checksum}")
            else:
                # プレフィックスが正しくない場合、またはデータが不十分な場合、最初のバイトを破棄して再試行
                self.buffer = self.buffer[start+1:] if start != -1 else self.buffer[1:]

def main(args=None):
    rclpy.init(args=args)
    node = SerialToPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
