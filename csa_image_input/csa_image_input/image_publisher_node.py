# image_publisher_node.py (Associations 기반 리팩토링)

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
import os, cv2, yaml
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time

def float_ts_to_ros_time(ts_float):
    sec = int(ts_float)
    nanosec = int((ts_float - sec) * 1e9)
    ros_time = Time()
    ros_time.sec = sec
    ros_time.nanosec = nanosec
    return ros_time

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mode', 'dataset'),
                ('dataset.rgb_path', ''),
                ('dataset.depth_path', ''),
                ('dataset.associations_path', ''),
                ('dataset.camera_info_path', ''),
                ('camera.rgb_device', 0),
                ('publish.fps', 30.0),
                ('publish.loop', True),
                ('publish.topic_rgb', '/camera/color/image_raw'),
                ('publish.topic_depth', '/camera/depth/image_raw'),
                ('publish.topic_info', '/camera/color/camera_info'),
            ]
        )
        self.bridge = CvBridge()
        self.timer = None
        self.frame_index = 0

        self.setup_mode()
        self.setup_publishers()
        self.setup_timer()

    def setup_mode(self):
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        if self.mode == 'dataset':
            self.rgb_dir = self.get_parameter('dataset.rgb_path').value
            self.depth_dir = self.get_parameter('dataset.depth_path').value
            self.associations_path = self.get_parameter('dataset.associations_path').value
            self.camera_info_path = self.get_parameter('dataset.camera_info_path').value
            self.loop = self.get_parameter('publish.loop').value

            self.associations = []
            if os.path.exists(self.associations_path):
                with open(self.associations_path, 'r') as f:
                    for line in f:
                        if line.startswith('#') or not line.strip():
                            continue
                        rgb_ts, rgb_file, depth_ts, depth_file = line.strip().split()
                        self.associations.append((rgb_ts, rgb_file, depth_ts, depth_file))
            else:
                self.get_logger().error("associations.txt not found.")
                rclpy.shutdown()

        elif self.mode == 'camera':
            self.rgb_device = self.get_parameter('camera.rgb_device').value
            self.cap = cv2.VideoCapture(self.rgb_device)
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
            rclpy.shutdown()

    def setup_publishers(self):
        qos = qos_profile_sensor_data
        self.pub_rgb = self.create_publisher(Image, self.get_parameter('publish.topic_rgb').value, qos)
        self.pub_depth = self.create_publisher(Image, self.get_parameter('publish.topic_depth').value, qos)
        self.pub_info = self.create_publisher(CameraInfo, self.get_parameter('publish.topic_info').value, qos)

        self.cam_info = CameraInfo()
        if self.mode == 'dataset' and os.path.exists(self.camera_info_path):
            cam_info_mgr = CameraInfoManager(node=self, cname='rgb_cam', url='file://' + self.camera_info_path)
            try:
                if cam_info_mgr.loadCameraInfo():
                    self.cam_info = cam_info_mgr.getCameraInfo()
                else:
                    self.get_logger().warn("⚠️ Failed to load camera info from YAML. Using default CameraInfo.")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Exception during camera info loading: {e}. Using default CameraInfo.")
        
        self.cam_info.header.stamp = self.get_clock().now().to_msg()
        self.cam_info.header.frame_id = "camera_link"
        self.pub_info.publish(self.cam_info)

    def setup_timer(self):
        fps = self.get_parameter('publish.fps').value
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)

    def publish_frame(self):
        if self.mode == 'dataset':
            if self.frame_index >= len(self.associations):
                if self.loop:
                    self.frame_index = 0
                else:
                    self.get_logger().info(f"📦 End of dataset at frame {self.frame_index}")
                    self.timer.cancel()  # ✅ 타이머 종료
                    return

            self.get_logger().info(f"📤 Publishing frame {self.frame_index}/{len(self.associations)}")
                
            rgb_ts, rgb_file, depth_ts, depth_file = self.associations[self.frame_index]
            rgb_path = os.path.join(self.rgb_dir, os.path.basename(rgb_file))
            depth_path = os.path.join(self.depth_dir, os.path.basename(depth_file))
            rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
            depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

            # 정확한 timestamp로 변환
            stamp = float_ts_to_ros_time(float(rgb_ts))

            # 메시지 생성 및 타임스탬프 부여
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = "camera_link"

            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
            depth_msg.header.stamp = stamp  # RGB와 동일하게 맞춤
            depth_msg.header.frame_id = "camera_link"

            self.cam_info.header.stamp = stamp  # 일반적으로 rgb 기준 사용

            # 퍼블리시
            self.pub_rgb.publish(rgb_msg)
            self.pub_depth.publish(depth_msg)
            self.pub_info.publish(self.cam_info)
            self.frame_index += 1

        elif self.mode == 'camera':
            ret, rgb = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to read from camera.")
                return

            now = self.get_clock().now().to_msg()
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = "camera_link"

            self.pub_rgb.publish(rgb_msg)
            self.pub_info.publish(self.cam_info)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
