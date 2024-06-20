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

    def listener_callback(self, msg):
        if len(msg.data) == 5:
            speed, direction, theta, team_color, action_number = msg.data
            speed = float(speed)
            direction = float(direction)
            theta = float(theta)
            team_color = int(team_color)
            action_number = int(action_number)
            send_data = struct.pack('>BBfffBB', 0xA5, 0xA5, speed, direction, theta, team_color, action_number)
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
