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
            # YAML에서 로드된 파라미터 값 확인 및 로깅 추가
            self.rgb_dir = self.get_parameter('dataset.rgb_path').get_parameter_value().string_value
            self.depth_dir = self.get_parameter('dataset.depth_path').get_parameter_value().string_value
            self.associations_path = self.get_parameter('dataset.associations_path').get_parameter_value().string_value
            self.camera_info_path = self.get_parameter('dataset.camera_info_path').get_parameter_value().string_value
            self.loop = self.get_parameter('publish.loop').value

            # 로드된 파라미터를 로그로 확인
            self.get_logger().info(f"✅ Dataset mode selected with parameters:\n"
                                    f"  rgb_path: {self.rgb_dir}\n"
                                    f"  depth_path: {self.depth_dir}\n"
                                    f"  associations_path: {self.associations_path}\n"
                                    f"  camera_info_path: {self.camera_info_path}")

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
            try:
                # CameraInfoManager는 첫 번째 인자로 Node 전달
                self.cam_info_mgr = CameraInfoManager(self, cname='rgb_cam', url='file://' + self.camera_info_path)
                if self.cam_info_mgr.loadCameraInfo():
                    self.cam_info = self.cam_info_mgr.getCameraInfo()
                    self.get_logger().info("✅ CameraInfo loaded successfully from YAML.")
                else:
                    self.get_logger().warn("⚠️ CameraInfo YAML load failed. Using default CameraInfo.")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Exception during CameraInfo loading: {e}. Using default CameraInfo.")
        else:
            if self.mode == 'dataset':
                self.get_logger().warn(f"⚠️ CameraInfo file does not exist: {self.camera_info_path}. Using default CameraInfo.")

        # frame_id은 고정
        self.cam_info.header.frame_id = "camera_link"
        # self.cam_info.header.frame_id = "camera_link"
        # self.pub_info.publish(self.cam_info)

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
                    self.timer.cancel()
                    return

            self.get_logger().info(f"📤 Publishing frame {self.frame_index}/{len(self.associations)}")
                
            rgb_ts, rgb_file, depth_ts, depth_file = self.associations[self.frame_index]
            rgb_path = os.path.join(self.rgb_dir, os.path.basename(rgb_file))
            depth_path = os.path.join(self.depth_dir, os.path.basename(depth_file))
            rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
            depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

            # ✅ 타임스탬프를 현재 시간으로 설정
            stamp = self.get_clock().now().to_msg()

            rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = "camera_link"

            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = "camera_link"

            self.cam_info.header.stamp = stamp

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
