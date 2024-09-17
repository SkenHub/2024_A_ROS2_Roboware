import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')

        # マイコンからの自己位置データの購読
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'robot_position',
            self.robot_position_callback,
            10
        )
        # 推定位置データのパブリッシャー
        self.publisher_ = self.create_publisher(Float32MultiArray, 'estimated_position', 10)

        # 初期化
        self.current_position = [0.0, 0.0, 0.0]
        self.realsense_position = [0.0, 0.0, 0.0]
        self.use_realsense = False  # RealSenseカメラの使用を示すフラグ
        self.realsense_weight = 0.6
        self.microcontroller_weight = 0.4

        # RealSenseの初期化
        try:
            self.pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.pose)
            self.pipe.start(cfg)
            self.use_realsense = True  # RealSenseカメラが接続されている場合
            self.get_logger().info('RealSense camera connected successfully.')
        except RuntimeError as e:
            self.get_logger().warn(f'RealSense camera not connected: {e}')

        # 定期的にデータを処理するためのタイマー
        self.create_timer(0.1, self.timer_callback)

    def robot_position_callback(self, msg):
        # マイコンからの自己位置データを更新
        microcontroller_position = msg.data
        self.get_logger().info(f"Received microcontroller position: {microcontroller_position}")
        self.compute_estimated_position(microcontroller_position)

    def compute_estimated_position(self, microcontroller_position):
        if self.use_realsense:
            # RealSenseとマイコンのデータを組み合わせて推定位置を計算
            self.current_position = [
                (self.realsense_position[0] * self.realsense_weight + microcontroller_position[0] * self.microcontroller_weight) / (self.realsense_weight + self.microcontroller_weight),
                (self.realsense_position[1] * self.realsense_weight + microcontroller_position[1] * self.microcontroller_weight) / (self.realsense_weight + self.microcontroller_weight),
                (self.realsense_position[2] * self.realsense_weight + microcontroller_position[2] * self.microcontroller_weight) / (self.realsense_weight + self.microcontroller_weight),
            ]
        else:
            # RealSenseがない場合、マイコンのデータのみを使用
            self.current_position = microcontroller_position

        self.get_logger().info(f"Estimated position: {self.current_position}")

        estimated_position_msg = Float32MultiArray(data=self.current_position)
        self.publisher_.publish(estimated_position_msg)

    def timer_callback(self):
        if self.use_realsense:
            # RealSenseカメラが使用可能な場合のみ、データを取得
            frames = self.pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            if pose:
                data = pose.get_pose_data()
                self.realsense_position = [data.translation.x * 1000, data.translation.y * 1000, data.rotation.z]
                self.get_logger().info(f"RealSense position: {self.realsense_position}")

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
