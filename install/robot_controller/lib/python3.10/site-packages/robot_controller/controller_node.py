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

        # 地点の座標を設定
        self.locations = {
            '1': [1500, 1500, 0],  # [x, y, theta]
            '2': [1500, 0, 0],
            '3': [0, 0, 0]
        }
        self.current_position = [0.0, 0.0, 0.0]  # 初期位置 [x, y, theta]
        self.max_speed = 500.0  # 最大速度 [mm/s]
        self.max_acceleration = 250.0  # 最大加速度 [mm/s^2]
        self.last_speed = 0.0  # 前回の速度 [mm/s]
        self.last_time = self.get_clock().now()  # 前回の時間

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
                self.move_to_target(self.current_position, float(team_color), float(i + 1))  # 現在の位置から動作番号を送信
                return

        # 指定された位置に移動する
        if any(positions):
            for i, pos in enumerate(positions):
                if pos == 1:
                    target = self.locations[str(i + 1)]
                    self.move_to_target(target, float(team_color), float(0))  # action_number is 0 when moving to positions
                    break

    def move_to_target(self, target, team_color, action_number):
        x, y, theta = target
        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)  # 距離をmmで計算
        direction = math.degrees(math.atan2(dy, dx)) % 360

        self.get_logger().info(f"Moving to target: {target} with direction {direction} and distance {distance}")

        # 現在の時間を取得
        current_time = self.get_clock().now()
        time_delta = (current_time - self.last_time).nanoseconds / 1e9  # 秒に変換

        # 速度を計算
        desired_speed = min(self.max_speed, distance / time_delta)
        acceleration = (desired_speed - self.last_speed) / time_delta

        if acceleration > self.max_acceleration:
            speed = self.last_speed + self.max_acceleration * time_delta
        elif acceleration < -self.max_acceleration:
            speed = self.last_speed - self.max_acceleration * time_delta
        else:
            speed = desired_speed

        self.last_speed = speed
        self.last_time = current_time

        self.send_velocity_command(speed, direction, theta, team_color, action_number)

    def send_velocity_command(self, speed, direction, theta, team_color, action_number):
        msg = Float32MultiArray()
        msg.data = [float(speed), float(direction), float(theta), float(team_color), float(action_number)]
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
