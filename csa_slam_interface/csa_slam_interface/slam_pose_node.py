import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
from pathlib import Path
import subprocess
import threading
import re

class ORBSLAMPosePublisher(Node):
    def __init__(self):
        super().__init__('slam_pose_node')
        self.get_logger().info("📡 [CSA] ORB-SLAM2 연동 Pose Publisher 초기화됨")

        # 실시간 퍼블리셔
        self.publisher = self.create_publisher(PoseStamped, '/camera/pose', QoSProfile(depth=10))
        self.frame_id = "map"

        # ORB-SLAM2 실행 명령
        self.orb_cmd = [
            "/home/jack/ORB_SLAM2/Examples/RGB-D/rgbd_tum",
            "/home/jack/ORB_SLAM2/Vocabulary/ORBvoc.txt",
            "/home/jack/ORB_SLAM2/Examples/RGB-D/TUM1.yaml",
            "/home/jack/ros2_ws/src/csa_image_input/dataset/rgbd_dataset_freiburg2_desk_with_person_secret/"
        ]

        # subprocess 실행
        self.process = subprocess.Popen(
            self.orb_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        # 백그라운드 파싱 스레드 실행
        self.thread = threading.Thread(target=self.parse_and_publish)
        self.thread.daemon = True
        self.thread.start()

    def parse_and_publish(self):
        # 출력에서 pose 패턴 추출 (예: 로그 형식은 커스터마이징 가능)
        pose_pattern = re.compile(
            r'(\d+\.\d+)\s+tx:([-.\d]+)\s+ty:([-.\d]+)\s+tz:([-.\d]+)\s+qx:([-.\d]+)\s+qy:([-.\d]+)\s+qz:([-.\d]+)\s+qw:([-.\d]+)'
        )

        for line in self.process.stdout:
            match = pose_pattern.search(line)
            if match:
                t, tx, ty, tz, qx, qy, qz, qw = map(float, match.groups())
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.pose.position.x = tx
                msg.pose.position.y = ty
                msg.pose.position.z = tz
                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw

                self.publisher.publish(msg)
                self.get_logger().info(f"📨 Pose published at t={t:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = ORBSLAMPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
