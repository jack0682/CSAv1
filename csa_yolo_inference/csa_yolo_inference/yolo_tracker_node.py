import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from csa_interfaces.msg import TrackedObject, TrackedObjectArray
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge


import numpy as np
from pathlib import Path
from .yolov5_wrapper import Yolov5Detector
from .strongsort_tracker import StrongSortTracker
from csa_utils.config_utils import load_ros2_params
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

class YoloTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')
        self.get_logger().info("🚀 [CSA] YOLO Tracker Node Started.")

        # Load config
        config_path = Path(__file__).parent.parent / 'config/global_parameters.yaml'
        self.params = load_ros2_params(str(config_path), self.get_name())

        # Initialize detector and tracker
        self.detector = Yolov5Detector(self.params['model'])
        self.tracker = StrongSortTracker(self.params['tracking'])

        # ROS
        self.bridge = CvBridge()
        self.frame_index = 0

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            TrackedObjectArray,
            '/tracked_objects',
            10
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Failed to convert image: {e}")
            return

        # 안정적인 타임스탬프 확보
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.get_logger().warn("⚠️ Image header.stamp is zero! Using current time instead.")
            stamp = self.get_clock().now().to_msg()
        else:
            stamp = msg.header.stamp

        # Detection + Tracking
        detections = self.detector.infer(frame)
        tracked_objs = self.tracker.update(detections, frame)

        msg_time = stamp.sec + stamp.nanosec * 1e-9
        now_time = self.get_clock().now().nanoseconds * 1e-9
        frame_latency = now_time - msg_time
        self.get_logger().info(f"⏱️ Frame latency: {frame_latency:.3f}s")

        # Publish
        tracked_msg = self.create_tracked_object_array_msg(tracked_objs, stamp)
        self.publisher.publish(tracked_msg)

        self.get_logger().info(f"🧠 Frame {self.frame_index} processed & published.")
        self.frame_index += 1


    def create_tracked_object_array_msg(self, tracked_objs, stamp=None):
        array_msg = TrackedObjectArray()

        # 안전한 타임스탬프
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        # 표준 헤더
        array_msg.header = Header()
        array_msg.header.stamp = stamp
        array_msg.header.frame_id = "camera_color_optical_frame"  # 실제 카메라 프레임과 일치

        # CSA 고유 필드
        array_msg.header_stamp = stamp
        array_msg.frame_id = self.frame_index

        for obj in tracked_objs:
            tobj = TrackedObject()
            tobj.label = obj.label
            tobj.class_id = obj.class_id
            tobj.confidence = float(obj.confidence)

            tobj.position = Point(x=obj.x3d, y=obj.y3d, z=obj.z3d)
            tobj.velocity = Vector3(x=obj.vx, y=obj.vy, z=obj.vz)

            tobj.x_min = int(obj.xmin)
            tobj.y_min = int(obj.ymin)
            tobj.x_max = int(obj.xmax)
            tobj.y_max = int(obj.ymax)

            tobj.track_id = obj.track_id
            tobj.frame_id = self.frame_index
            tobj.stamp = stamp

            # feature 예외 처리
            tobj.feature = np.array(obj.feature, dtype=np.float32).tolist() if obj.feature is not None else []
            tobj.relations = obj.relations if hasattr(obj, 'relations') else []

            array_msg.objects.append(tobj)

        return array_msg




def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
