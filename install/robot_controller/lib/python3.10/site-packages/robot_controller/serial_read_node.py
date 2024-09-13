import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialReadNode(Node):
    def __init__(self):
        super().__init__('serial_read_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  
        self.create_timer(0.1, self.read_data_from_mcu)

    def read_data_from_mcu(self):
        HEADER = b'\xA5\xA5'
        while self.serial_port.in_waiting > 0:
            buffer = b''
            while self.serial_port.in_waiting > 0:
                buffer += self.serial_port.read(1)
                if len(buffer) >= 2 and buffer[-2:] == HEADER:
                    data = self.serial_port.read(8)
                    if len(data) == 8:
                        action_number, mode, X, Y, theta = struct.unpack('>BBhhh', data)
                        msg = Float32MultiArray(data=[X, Y, theta])
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Received data: Action={action_number}, Mode={mode}, X={X}, Y={Y}, Theta={theta}")
                    buffer = b''

def main(args=None):
    rclpy.init(args=args)
    node = SerialReadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
