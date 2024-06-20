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
        self.timer = self.create_timer(0.1, self.timer_callback)  # Increase the frequency to check buffer more often
        self.buffer = b''

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)
        
        while len(self.buffer) >= 16:
            if self.buffer[:2] == b'\xA5\xA5':
                data = self.buffer[2:16]
                self.buffer = self.buffer[16:]
                if len(data) == 14:
                    position_data = struct.unpack('fffBB', data)

                    position_msg = Float32MultiArray()
                    position_msg.data = [
                        float(position_data[0]),  # x
                        float(position_data[1]),  # y
                        float(position_data[2]),  # theta
                        float(position_data[3]),  # team color
                        float(position_data[4])  # action number
                    ]

                    self.publisher_.publish(position_msg)
                    self.get_logger().info(f"Published position data: {position_msg.data}")
                else:
                    self.get_logger().error(f"Received incorrect data length: {len(data)}")
            else:
                self.get_logger().error(f"Invalid prefix: {self.buffer[:2].hex()}")
                self.buffer = self.buffer[1:]  # Drop the first byte and try again

def main(args=None):
    rclpy.init(args=args)
    node = SerialToPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
