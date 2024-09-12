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

        # 自己位置を更新するためのサブスクリプション
        self.position_subscription = self.create_subscription(
            Float32MultiArray,
            'robot_position',
            self.update_position_callback,
            10)

        # 地点の座標を設定
        self.locations = {
            '1': [500, 500, 0],  # [x, y, theta]
            '2': [0, 500, 0],
            '3': [0, 0, 0]
        }
        self.current_position = [0.0, 0.0, 0.0]  # 初期位置 [x, y, theta]
        self.max_speed = 500.0  # 最大速度 [mm/s]
        self.max_accel = 50.0  # 最大加速度 [mm/s^2]
        self.max_angular_speed = 30.0  # 最大角速度 [deg/s]

    def update_position_callback(self, msg):
        self.current_position = [msg.data[0], msg.data[1], msg.data[2]]
        self.get_logger().info(f"Updated current position: {self.current_position}")

    def listener_callback(self, msg):
        data = msg.data.split(',')
        actions = list(map(int, data[:5]))
        positions = list(map(int, data[5:8]))
        team_color = int(data[8])
        emergency_stop = int(data[9])

        self.get_logger().info(f"Received data: {data}")

        # 非常停止処理
        if emergency_stop == 1:
            self.send_velocity_command(0.0, 0.0, 0.0, float(team_color), float(255))
            return

        # 指定された動作番号に基づいて動作を行う
        for i, action in enumerate(actions):
            if action == 1:
                self.move_to_target(self.current_position, float(team_color), float(i + 1))
                return

        # 指定された位置に移動する
        if any(positions):
            for i, pos in enumerate(positions):
                if pos == 1:
                    target = self.locations[str(i + 1)]
                    self.move_to_target(target, float(team_color), float(0))
                    break

    def move_to_target(self, target, team_color, action_number):
        x, y, target_theta = target
        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)  # 距離をミリメートル単位で計算
        direction = (math.degrees(math.atan2(dy, -dx)) - 90) % 360  # 方向を0-360度に設定

        self.get_logger().info(f"Moving to target: {target} with direction {direction} and distance {distance}")

        if distance < 50:  # 例えば50mm以下になったら停止
            speed = 0.0
        else:
            speed = min(self.max_speed, distance)

        # 角速度を計算
        dtheta = (target_theta - self.current_position[2] + 360) % 360
        if dtheta > 180:
            dtheta -= 360
        angular_speed = max(min(dtheta, self.max_angular_speed), -self.max_angular_speed)

        self.send_velocity_command(speed, direction, angular_speed, team_color, action_number)

    def send_velocity_command(self, speed, direction, angular_speed, team_color, action_number):
        msg = Float32MultiArray()
        msg.data = [float(action_number), float(team_color), float(speed), float(direction), float(angular_speed)]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent velocity command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
