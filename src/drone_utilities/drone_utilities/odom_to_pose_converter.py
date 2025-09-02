#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OdomToPoseConverter(Node):
    """
    这个节点订阅一个Odometry话题，并将其中的位姿信息
    提取出来，以PoseStamped的格式重新发布。
    """
    def __init__(self):
        super().__init__('odom_to_pose_converter')

        # 声明参数，允许在launch文件中灵活配置话题名
        self.declare_parameter('odom_in_topic', '/T265/pose/sample')
        self.declare_parameter('pose_out_topic', '/mavros/vision_pose/pose')

        # 从参数服务器获取话题名称
        odom_in_topic = self.get_parameter('odom_in_topic').get_parameter_value().string_value
        pose_out_topic = self.get_parameter('pose_out_topic').get_parameter_value().string_value

        # 创建QoS配置，与T265发布者匹配 (Best Effort)
        subscriber_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 创建订阅者
        self.subscription = self.create_subscription(
            Odometry,
            odom_in_topic,
            self.odom_callback,
            subscriber_qos_profile)

        # 创建发布者
        self.publisher = self.create_publisher(
            PoseStamped,
            pose_out_topic,
            10)

        self.get_logger().info(f"转换节点已启动. 订阅: '{odom_in_topic}', 发布: '{pose_out_topic}'")

    def odom_callback(self, odom_msg: Odometry):
        # 创建一个新的PoseStamped消息
        pose_msg = PoseStamped()

        # 1. 复制消息头（包含时间戳和frame_id），并修改frame_id
        pose_msg.header = odom_msg.header
        pose_msg.header.frame_id = "odom"  # 修改为"odom"

        # 2. 复制位姿信息
        pose_msg.pose = odom_msg.pose.pose

        # 发布转换后的消息
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    converter_node = OdomToPoseConverter()
    try:
        rclpy.spin(converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        converter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()