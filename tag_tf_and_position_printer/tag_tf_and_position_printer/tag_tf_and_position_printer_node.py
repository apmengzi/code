import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from apriltag_msgs.msg import AprilTagDetectionArray
import math
import os

class TagOdomLogger(Node):
    def __init__(self):
        super().__init__('tag_odom_logger')

        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅 AprilTag 识别信息
        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )

        # 设置日志文件路径
        self.log_file = os.path.join(os.path.expanduser('/home/pi/qqq'), 'tag_base_in_odom.txt')

    def detection_callback(self, msg):
        if not msg.detections:
            return

        try:
            # 查找 base_link -> odom 的变换
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base', rclpy.time.Time())
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # 欧拉角 yaw（绕 Z 轴旋转）
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            # 更新每个检测到的 tag 的信息
            tag_dict = self.read_log()
            for det in msg.detections:
                tag_id = det.id
                tag_dict[f"tag_{tag_id}"] = f"x={x:.3f}, y={y:.3f}, theta={theta:.3f}"

            self.write_log(tag_dict)
            # self.get_logger().info("Updated tag position in odom")

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def read_log(self):
        """读取已有日志，返回字典"""
        data = {}
        if os.path.exists(self.log_file):
            with open(self.log_file, 'r') as f:
                for line in f:
                    if ':' in line:
                        tag, val = line.strip().split(':', 1)
                        data[tag.strip()] = val.strip()
        return data

    def write_log(self, tag_dict):
        """将更新后的数据写回日志文件"""
        with open(self.log_file, 'w') as f:
            for tag_id, val in tag_dict.items():
                f.write(f"{tag_id}: {val}\n")

def main(args=None):
    rclpy.init(args=args)
    node = TagOdomLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
