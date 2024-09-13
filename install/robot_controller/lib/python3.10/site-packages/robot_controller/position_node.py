import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')  # ノード名
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'robot_position',
            self.robot_position_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'estimated_position', 10)

        self.current_position = [0.0, 0.0, 0.0]  # 初期位置 [x, y, theta]
        self.realsense_position = [0.0, 0.0, 0.0]
        self.realsense_weight = 0
        self.microcontroller_weight = 1

        # RealSenseの初期化
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        self.pipe.start(cfg)

        self.create_timer(0.1, self.timer_callback)

    def robot_position_callback(self, msg):
        # マイコンから送られてきた自己位置
        microcontroller_position = msg.data
        self.get_logger().info(f"Received microcontroller position: {microcontroller_position}")

        # 重み付き平均を計算
        self.current_position = [
            (self.realsense_position[0] * self.realsense_weight + microcontroller_position[0] * self.microcontroller_weight) / (self.realsense_weight + self.microcontroller_weight),
            (self.realsense_position[1] * self.realsense_weight + microcontroller_position[1] * self.microcontroller_weight) / (self.realsense_weight + self.microcontroller_weight),
            (self.realsense_position[2] * self.realsense_weight + microcontroller_position[2] * self.microcontroller_weight) / (self.realsense_weight + self.microcontroller_weight),
        ]
        self.get_logger().info(f"Estimated position: {self.current_position}")

        # 推定位置をパブリッシュ
        estimated_position_msg = Float32MultiArray()
        estimated_position_msg.data = self.current_position
        self.publisher_.publish(estimated_position_msg)

    def timer_callback(self):
        # RealSenseからデータを取得
        frames = self.pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if pose:
            data = pose.get_pose_data()
            self.realsense_position = [
                data.translation.x * 1000,  # 単位をmmに変換
                data.translation.y * 1000,
                data.rotation.z * 180 / 3.14159  # ラジアンを度に変換
            ]
            self.get_logger().info(f"RealSense position: {self.realsense_position}")

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
