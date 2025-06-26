import sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge

# Avoid circular imports by moving imports inside the function
from csa_utils.config_utils import load_ros2_params
from .yolov5_wrapper import Yolov5Detector
from .strongsort_tracker import StrongSortTracker

# Ensure parent path added
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

class YoloTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')

        # Import log functions here to avoid circular imports
        from csa_utils.log import log_info, log_warn, log_error
        log_info(self, "🚀 [YOLO-TRACKER] Node initialized.")

        # Load parameters
        self.config = self.load_config()
        
        # Load YOLOv5 model and tracking parameters from config
        self.detector = Yolov5Detector(self.config['model']['yolo_weights'], 
                                       self.config['model']['conf_threshold'], 
                                       self.config['model']['iou_threshold'])
        self.tracker = StrongSortTracker(self.config['tracking'])
        
        self.bridge = CvBridge()
        self.frame_index = 0

        # ROS2 interface setup
        self.setup_ros_interfaces()

    def load_config(self):
        config_path = Path(__file__).parent.parent / 'config/global_parameters.yaml'
        try:
            config = load_ros2_params(str(config_path), self.get_name())
            log_info(self, f"✅ Config loaded: {config_path}")
            return config
        except Exception as e:
            from csa_utils.log import log_error
            log_error(self, f"❌ Error loading config: {e}")
            return {}

    def setup_ros_interfaces(self):
        # Subscribe to the image topic
        self.image_sub = self.create_subscription(
            Image,
            self.config.get('image_topic', '/camera/color/image_raw'),
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publisher for tracked objects
        self.publisher = self.create_publisher(
            TrackedObjectArray,
            '/tracked_objects',
            10
        )
        log_info(self, "📡 ROS2 interfaces (subscriber & publisher) set up.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            from csa_utils.log import log_error
            log_error(self, f"❌ CV Bridge error: {e}")
            return

        # Handle invalid timestamps
        image_stamp = msg.header.stamp
        if image_stamp.sec == 0 and image_stamp.nanosec == 0:
            from csa_utils.log import log_warn
            log_warn(self, "⚠️ Image header.stamp is zero. Using current time.")
            image_stamp = self.get_clock().now().to_msg()

        try:
            # Perform detection and tracking
            detections = self.detector.infer(frame)
            tracked_objs = self.tracker.update(detections, frame)
        except Exception as e:
            from csa_utils.log import log_error
            log_error(self, f"❌ Detection or Tracking failure: {e}")
            return

        # Optional Debug Visualization
        if self.config.get("debug_visualization", False):
            self.debug_visualization(frame, tracked_objs)

        # Calculate frame latency
        latency = (self.get_clock().now().nanoseconds * 1e-9) - \
                  (image_stamp.sec + image_stamp.nanosec * 1e-9)
        log_info(self, f"⏱️ Frame latency: {latency:.3f} sec")

        # Publish tracked objects
        self.publish_tracked_objects(tracked_objs, image_stamp)

    def debug_visualization(self, frame, tracked_objs):
        from csa_utils.visualization import draw_tracking_overlay
        debug_img = draw_tracking_overlay(frame, tracked_objs)
        # TODO: Optionally, display or save the debug image
        # e.g., using OpenCV to display or save the image: cv2.imshow("Debug Image", debug_img)

    def publish_tracked_objects(self, tracked_objs, stamp):
        msg = self.create_tracked_object_array_msg(tracked_objs, stamp)
        self.publisher.publish(msg)
        log_info(self, f"📤 Published frame {self.frame_index} with {len(tracked_objs)} objects.")
        self.frame_index += 1

    def create_tracked_object_array_msg(self, tracked_objs, stamp):
        from csa_interfaces.msg import TrackedObject, TrackedObjectArray
        array_msg = TrackedObjectArray()
        array_msg.header.stamp = stamp
        array_msg.header.frame_id = self.config.get('frame_id', "camera_color_optical_frame")
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
            tobj.feature = list(np.asarray(obj.feature, dtype=np.float32)) if obj.feature is not None else []
            tobj.relations = obj.relations if hasattr(obj, 'relations') else []

            array_msg.objects.append(tobj)

        return array_msg

def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        from csa_utils.log import log_warn
        log_warn(node, "🛑 [YOLO-TRACKER] Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        from csa_utils.log import log_info
        log_info(node, "✅ Node shutdown successfully.")
