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
            Float32MultiArray,'estimated_position',self.update_position_callback, 10)

        # 地点の座標を設定
        self.locations_normal = {
            '1': [1500, 1500, 0],  # [x, y, theta]
            '2': [0, 1500, 0],
            '3': [0, 0, 0],
            '4': [3000, 1500, 0],
            '5': [3000, 1500, 90]
        }
        self.locations_inverted = {
            '1': [-1500, 1500, 0],  # [x, y, theta]
            '2': [0, 1500, 0],
            '3': [0, 0, 0],
            '4': [-3000, 1500, 0],
            '5': [0, 1500, 90]
        }
        self.current_position = [0.0, 0.0, 0.0]  # 初期位置 [x, y, theta]
        self.max_speed = 800.0  # ノーマルモード最大速度 [mm/s]
        self.max_accel = 100.0  # 最大加速度 [mm/s^2]
        self.max_angular_speed = 10.0  # 最大角速度 [deg/s]
        self.mode = 0  # モード初期値
        self.speedmode = 0 #スピードモード

    def update_position_callback(self, msg):
        self.current_position = [msg.data[0], msg.data[1], msg.data[2]]
     #   self.get_logger().info(f"Updated current position: {self.current_position}")

    def listener_callback(self, msg):
        data = msg.data.split(',')
    
        # データの長さが最低11個あるか確認する
        if len(data) < 13:
            self.get_logger().error(f"受信データの長さが不足しています: {len(data)}個の要素があり、最低でも13個が必要です")
            return
    
        behavior = int(data[0])
        mode = int(data[1])
        buttons = list(map(int, data[2:6]))
        color = data[6]
        emergency_stop = int(data[7])
        lx = int(data[8])
        ly = int(data[9])
        rx = int(data[10])
        ry = int(data[11])
        self.speedmode = int(data[12])

        # 非常停止処理
        if emergency_stop == 1:
            self.send_velocity_command(0.0, 0.0, 0.0, mode, float(255))
            return

        # 手動操作モードの場合
        elif mode == 1:
          if not behavior == 3:
            if self.speedmode == 0:
             self.nx =5
             self.ny =5
            else:
             self.nx =0
             self.ny =100
            Vx = (rx-105)*self.nx
            Vy = (ry-107)*self.ny
            omega = (lx-102)/2
            self.send_velocity_command(Vx, Vy, omega, mode, behavior)
            self.get_logger().info(f"Vx={Vx}, Vy={Vy}, Omega={omega} ")
          else:
            Vx = 0
            Vy = 0
            omega = 0
            self.send_velocity_command(Vx, Vy, omega, mode, behavior)
            self.get_logger().info(f"Vx={Vx}, Vy={Vy}, Omega={omega} ")
              
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
        if self.speedmode == 0:
            self.max_speed = 500
        else:
            self.max_speed = 800

        x, y, target_theta = target
        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        direction = (math.degrees(math.atan2(dy, dx)) - 90) % 360

        # 現在の速度に基づいて目標速度を制限
        current_speed = math.sqrt(self.current_position[0]**2 + self.current_position[1]**2)  # 現在の速度

        # 目標速度と加速度制限を考慮して速度を決定
##############################  目標速度と加速度制限を考慮して速度を決定  #########################################################################
        desired_speed = min(self.max_speed, distance)
        if desired_speed > current_speed + self.max_accel:
            desired_speed = current_speed + self.max_accel
        elif desired_speed < current_speed - self.max_accel:
            desired_speed = current_speed - self.max_accel
#####################################################################################################################
        Vx = desired_speed * math.sin(math.radians(direction)) * -1
        Vy = desired_speed * math.cos(math.radians(direction))

        dtheta = (target_theta - self.current_position[2] + 360) % 360
        if dtheta > 180:
            dtheta -= 360
        omega = max(min(dtheta, self.max_angular_speed), -self.max_angular_speed)

        self.send_velocity_command(Vx, Vy, omega, team_color, action_number)

        self.get_logger().info(f"target:{target}  0={self.current_position[0]} 1={self.current_position[1]} 2={self.current_position[2]} Vx={Vx}, Vy={Vy}, Omega={omega}")

    def send_velocity_command(self, Vx, Vy, omega, team_color, action_number):
        msg = Float32MultiArray()
        msg.data = [float(Vx), float(Vy), float(omega), float(team_color), float(action_number)]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
