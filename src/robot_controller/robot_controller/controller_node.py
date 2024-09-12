import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(
            String,
            'web_socket_pub',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'cmd_vel', 10)
        self.position_subscription = self.create_subscription(
            Float32MultiArray,
            'estimated_position',
            self.update_position_callback,
            10)

        # 地点の座標を設定
        self.locations_normal = {
            '1': [500, 500, 0],  # [x, y, theta]
            '2': [0, 500, 0],
            '3': [0, 0, 0]
        }
        self.locations_inverted = {
            '1': [-500, 500, 0],  # [x, y, theta]
            '2': [0, 500, 0],
            '3': [0, 0, 0]
        }
        self.current_position = [0.0, 0.0, 0.0]  # 初期位置 [x, y, theta]
        self.max_speed = 500.0  # 最大速度 [mm/s]
        self.max_accel = 50.0  # 最大加速度 [mm/s^2]
        self.max_angular_speed = 30.0  # 最大角速度 [deg/s]
        self.mode = 0  # モード初期値

    def update_position_callback(self, msg):
        self.current_position = [msg.data[0], msg.data[1], msg.data[2]]
        self.get_logger().info(f"Updated current position: {self.current_position}")

    def listener_callback(self, msg):
        data = msg.data.split(',')
        behavior = int(data[0])
        mode = int(data[1])
        buttons = list(map(int, data[2:5]))
        color = data[5]
        emergency_stop = int(data[6])
        lx = int(data[7])
        ly = int(data[8])
        rx = int(data[9])
        ry = int(data[10])

        self.get_logger().info(f"Received data: {data}")

        # 非常停止処理
        if emergency_stop == 1:
            self.send_velocity_command(0.0, 0.0, 0.0, mode, float(255))
            return

        # 手動操作モードの場合
        if mode == 1:
            Vx = ly
            Vy = lx
            omega = rx
            self.send_velocity_command(Vx, Vy, omega, mode, behavior)
        else:
            # ボタン設定に基づいてターゲットを決定
            if any(buttons):
                for i, button in enumerate(buttons):
                    if button == 1:
                        if color == '0':
                            target = self.locations_normal[str(i + 1)]
                        else:
                            target = self.locations_inverted[str(i + 1)]
                        self.move_to_target(target, float(mode), float(0))
                        break

    def move_to_target(self, target, team_color, action_number):
        x, y, target_theta = target
        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        direction = (math.degrees(math.atan2(dy, dx)) - 90) % 360

        self.get_logger().info(f"Moving to target: {target} with direction {direction} and distance {distance}")

        if distance < 50:
            Vx = 0.0
            Vy = 0.0
        else:
            Vx = min(self.max_speed, distance) * math.cos(math.radians(direction))
            Vy = min(self.max_speed, distance) * math.sin(math.radians(direction))

        dtheta = (target_theta - self.current_position[2] + 360) % 360
        if dtheta > 180:
            dtheta -= 360
        omega = max(min(dtheta, self.max_angular_speed), -self.max_angular_speed)

        self.send_velocity_command(Vx, Vy, omega, team_color, action_number)

    def send_velocity_command(self, Vx, Vy, omega, team_color, action_number):
        msg = Float32MultiArray()
        msg.data = [float(Vx), float(Vy), float(omega), float(team_color), float(action_number)]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent velocity command: Vx={Vx}, Vy={Vy}, Omega={omega}, Team Color={team_color}, Action Number={action_number}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
