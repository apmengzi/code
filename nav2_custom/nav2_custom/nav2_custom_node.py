import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
import time
from builtin_interfaces.msg import Time  # 加在文件顶部

class Nav2CustomNode(Node):
    def __init__(self):
        super().__init__('nav2_custom_node')
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Subscription for amcl_pose
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.amcl_pose_received = False
        # Action client for NavigateThroughPoses
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')

    def amcl_pose_callback(self, msg):
        if not self.amcl_pose_received:
            self.get_logger().info('Received first amcl_pose')
            self.amcl_pose_received = True

    def run(self):
        # Step 1: Prepare initial pose
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        # 使用时间戳 0，更保险
        initial_pose.header.stamp = Time(sec=0, nanosec=0)
        initial_pose.pose.pose.position.x = 0.1
        initial_pose.pose.pose.position.y = 0.05
        initial_pose.pose.pose.orientation.w = 1
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685389
        ]

        # Step 2: 发布多次初始位姿（AMCL 有时候漏收）
        for i in range(10):
            self.initial_pose_pub.publish(initial_pose)
            self.get_logger().info(f"[{i+1}/5] Published initial pose")
            time.sleep(1.2)

        # Step 3: 等待 /amcl_pose 回调 + 设置超时机制
        self.get_logger().info("Waiting for amcl_pose...")
        start_time = time.time()
        timeout = 20.0
        while rclpy.ok() and not self.amcl_pose_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout: Did not receive amcl_pose within 20 seconds.")
                return

        # Step 4: 等待 5 秒
        self.get_logger().info('Received amcl_pose. Waiting 5 seconds...')
        time.sleep(3)

        # Step 5: 等待导航服务器
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigate_through_poses action server...')
            if not rclpy.ok():
                return

        # Step 6: 构造目标路径
        goal_msg = NavigateThroughPoses.Goal()

        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.header.stamp = self.get_clock().now().to_msg()
        pose1.pose.position.x = float(0.96)
        pose1.pose.position.y = float(-0.24)
        pose1.pose.orientation.w = float(0.94)
        pose1.pose.orientation.z = float(0.33)
        goal_msg.poses.append(pose1)

        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.header.stamp = self.get_clock().now().to_msg()
        pose2.pose.position.x = float(2.24)
        pose2.pose.position.y = float(-1.5)
        pose2.pose.orientation.w = float(0.93)
        pose2.pose.orientation.z =  float(-0.35)
        goal_msg.poses.append(pose2)

        pose3 = PoseStamped()
        pose3.header.frame_id = 'map'
        pose3.header.stamp = self.get_clock().now().to_msg()
        pose3.pose.position.x = float(3.42)
        pose3.pose.position.y = float(-1.49)
        pose3.pose.orientation.w = float(0.99)
        pose3.pose.orientation.z = float(-0.11)
        goal_msg.poses.append(pose3)

        pose4 = PoseStamped()
        pose4.header.frame_id = 'map'
        pose4.header.stamp = self.get_clock().now().to_msg()
        pose4.pose.position.x = float(0.0)
        pose4.pose.position.y = float(0.0)
        pose4.pose.orientation.w = float(1.0)
        pose4.pose.orientation.z = float(0.0)
        goal_msg.poses.append(pose4)

        # Step 7: 发送导航目标
        self.get_logger().info('Sending navigate_through_poses goal')
        self.nav_through_poses_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2CustomNode()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()