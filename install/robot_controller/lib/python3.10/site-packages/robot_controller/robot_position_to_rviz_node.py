import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

class RobotPositionToRVizNode(Node):
    def __init__(self):
        super().__init__('robot_position_to_rviz_node')

        # サブスクライバーを作成し、/estimated_positionトピックからデータを購読
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'estimated_position',
            self.position_callback,
            10
        )

        # /robot_poseトピックにPoseStampedメッセージをパブリッシュするパブリッシャーを作成
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'robot_pose',
            10
        )

    def position_callback(self, msg):
        # Float32MultiArrayからx, y, thetaを取得
        x, y, theta = msg.data

        # PoseStampedメッセージを作成
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"  # RVizのフレームを "map" に設定

        # ポジション情報を設定
        pose.pose.position.x = x / 1000.0  # mmをmに変換
        pose.pose.position.y = y / 1000.0  # mmをmに変換
        pose.pose.position.z = 0.0

        # オリエンテーションを設定 (Euler角からクォータニオンに変換)
        q = quaternion_from_euler(0, 0, math.radians(theta))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        # メッセージをパブリッシュ
        self.publisher_.publish(pose)
        self.get_logger().info(f"Published robot pose: x={pose.pose.position.x}, y={pose.pose.position.y}, theta={theta}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionToRVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
