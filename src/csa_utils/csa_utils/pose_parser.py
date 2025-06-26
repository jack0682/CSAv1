#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
import time

class PoseParserNode(Node):
    def __init__(self):
        super().__init__('pose_parser_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/camera/pose', qos_profile_sensor_data)
        self.get_logger().info("📌 [CSA] pose_parser_node 시작됨")

        self.file_path = '/home/jack/ORB_SLAM2/CameraTrajectory.txt'
        self.publish_rate = self.create_rate(30)  # 30Hz

        self.parse_and_publish()

    def parse_and_publish(self):
        try:
            with open(self.file_path, 'r') as file:
                for line in file:
                    if line.strip() == "":
                        continue  # 빈 줄은 skip
                    msg = self.line_to_pose(line)
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"🛰️ 퍼블리시됨: {msg}")
                    self.publish_rate.sleep()
        except FileNotFoundError:
            self.get_logger().error(f"❌ 파일을 찾을 수 없습니다: {self.file_path}")
        except Exception as e:
            self.get_logger().error(f"🚨 예외 발생: {e}")

    def line_to_pose(self, line):
        parts = list(map(float, line.strip().split()))
        timestamp, tx, ty, tz, qx, qy, qz, qw = parts

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # 또는 'odom', 'world'

        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.position.z = tz

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

def main(args=None):
    rclpy.init(args=args)
    node = PoseParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
