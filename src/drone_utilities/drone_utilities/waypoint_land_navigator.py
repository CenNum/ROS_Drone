#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import time

# --- 在这里定义您的航点 ---
WAYPOINTS = [
    (1.0, 0.0, 1.5),    # 目标1: 原点前方1米, 高度1.5米
    (1.0, 1.0, 1.5),    # 目标2: 在目标1的基础上, 向左1米
    (0.0, 1.0, 1.5),    # 目标3: 在目标2的基础上, 向后1米
    (0.0, 0.0, 1.5)     # 目标4: 飞回原点
]
WAYPOINT_TOLERANCE = 0.2 # 到达航点的容忍距离 (米)

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_land_navigator_node')
        self.current_state = None
        self.current_pose = None
        self.waypoint_index = 0
        self.mission_started = False
        self.land_command_sent = False # ‼️ 新增：用于标记是否已发送降落指令

        # QoS 配置
        subscriber_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        # 订阅者
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, subscriber_qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, subscriber_qos_profile)
        
        # 发布者
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # ‼️ 新增：创建服务客户端，用于发送降落指令
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # 创建一个20Hz的定时器来运行主控制循环
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("航点降落导航节点已启动。")
        self.get_logger().info("请手动起飞并稳定悬停，然后将飞行模式切换到 GUIDED 以开始任务。")

    def state_callback(self, msg: State):
        self.current_state = msg
        if msg.mode == "GUIDED" and not self.mission_started:
            self.start_mission()

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def start_mission(self):
        if self.mission_started or self.current_pose is None:
            return
        self.mission_started = True
        self.waypoint_index = 0
        self.get_logger().info(f"检测到GUIDED模式，开始执行航点任务！飞往航点 #{self.waypoint_index + 1}")

    def send_land_command(self):
        # 等待服务可用
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('降落指令服务不可用, 正在等待...')
            return

        request = CommandLong.Request()
        request.command = 21  # MAV_CMD_NAV_LAND 的ID
        request.param1 = 0.0  # Abort landing
        request.param2 = 0.0  # Precision land mode
        request.param3 = 0.0  # Empty
        request.param4 = 0.0  # Yaw angle
        request.param5 = 0.0  # Latitude
        request.param6 = 0.0  # Longitude
        request.param7 = 0.0  # Altitude

        future = self.cmd_client.call_async(request)
        self.get_logger().info("降落指令已发送。")

    def control_loop(self):
        if not self.mission_started or self.current_pose is None or self.current_state is None:
            return
        if self.current_state.mode != "GUIDED":
            self.mission_started = False
            self.land_command_sent = False # 重置降落状态
            self.get_logger().warn("已切出GUIDED模式，任务暂停。")
            return

        # 检查是否已完成所有航点
        if self.waypoint_index >= len(WAYPOINTS):
            # ‼️ 核心修改：如果任务完成且尚未发送降落指令，则发送指令
            if not self.land_command_sent:
                self.get_logger().info("所有航点已完成，发送降落指令。")
                self.send_land_command()
                self.land_command_sent = True
                # 停止定时器，因为任务已完全结束
                self.timer.cancel()
            return

        # --- 以下为正常的航点跟踪逻辑 ---
        target_waypoint = WAYPOINTS[self.waypoint_index]
        dx = self.current_pose.pose.position.x - target_waypoint[0]
        dy = self.current_pose.pose.position.y - target_waypoint[1]
        dz = self.current_pose.pose.position.z - target_waypoint[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if distance < WAYPOINT_TOLERANCE:
            self.get_logger().info(f"已到达航点 #{self.waypoint_index + 1}")
            self.waypoint_index += 1
            if self.waypoint_index >= len(WAYPOINTS):
                return # 进入下一次循环来触发降落逻辑
            else:
                self.get_logger().info(f"飞往下一个航点 #{self.waypoint_index + 1}")

        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "odom"
        target_pose.pose.position.x = float(target_waypoint[0])
        target_pose.pose.position.y = float(target_waypoint[1])
        target_pose.pose.position.z = float(target_waypoint[2])
        target_pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(target_pose)

def main(args=None):
    rclpy.init(args=args)
    navigator_node = WaypointNavigator()
    try:
        rclpy.spin(navigator_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()