import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from csa_interfaces.msg import TrackedObjectArray

import yaml
from pathlib import Path

from csa_semantic_mapper.semantic_memory import SemanticMemory
from csa_semantic_mapper.scene_graph_builder import SceneGraphBuilder
from csa_utils.coordinate_transform import project_bbox_to_3d
from csa_utils.config_utils import load_ros2_params
from cv_bridge import CvBridge
from message_filters import Subscriber
from rclpy.qos import qos_profile_sensor_data

class SemanticMapperNode(Node):
    def __init__(self):
        super().__init__('semantic_mapper_node')
        self.get_logger().info("🧠 [CSA] Semantic Mapper Node Initialized.")

        # Load YAML config
        config_path = Path(__file__).parent.parent / 'config/semantic_mapper.yaml'
        self.params = load_ros2_params(str(config_path), self.get_name()) 
        buffer_size = self.params['frame_buffer']['max_size']

        # Initialize modules
        self.semantic_memory = SemanticMemory(self.params)
        self.scene_graph_builder = SceneGraphBuilder(self.params)
        self.bridge = CvBridge()
        
        # Subscribers
        self.sub_depth = Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.sub_tracked = Subscriber(self, TrackedObjectArray, '/tracked_objects', qos_profile=qos_profile_sensor_data)
        self.sub_pose = Subscriber(self, PoseStamped, '/camera/pose', qos_profile=qos_profile_sensor_data)

        # Time synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_tracked, self.sub_pose, self.sub_depth],
            queue_size=30,
            slop=3.0,
            allow_headerless=True  # ✅ 추가
        )
        self.sync.registerCallback(self.synced_callback)

        self.frame_count = 0
        self.get_logger().info("🟢 Semantic Mapper Init Complete. Waiting for synced_callback...")


    def load_yaml(self, path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)
        
    def append_graph_to_jsonl(self, graph, frame_id, save_path, logger):
        try:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            with open(save_path, 'a') as f:
                json.dump(graph, f, ensure_ascii=False)
                f.write('\n')
            logger.info(f"✅ [SceneGraph] Frame {frame_id} appended to {save_path}")
        except Exception as e:
            logger.error(f"❌ Failed to write scene graph to JSONL: {e}")

    def synced_callback(self, tracked_msg, pose_msg, depth_msg):
        camera_pose = pose_msg.pose
        timestamp = tracked_msg.header.stamp
        self.get_logger().info(f"🧠 Synced callback triggered at frame {self.frame_count}")

        # Debugging timestamp match
        t_pose = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9
        t_tracked = tracked_msg.header.stamp.sec + tracked_msg.header.stamp.nanosec * 1e-9
        t_depth = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f"📥 [SYNCED] Received pose + tracked_objects + depth | t_pose={t_pose:.6f}, t_tracked={t_tracked:.6f}, t_depth={t_depth:.6f}, Δt={abs(t_pose - t_tracked):.6f}")

        
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"❌ Failed to convert depth image: {e}")
            return

        for obj in tracked_msg.objects:
            x3d, y3d, z3d = project_bbox_to_3d(
                bbox=(obj.x_min, obj.y_min, obj.x_max, obj.y_max),
                camera_pose=camera_pose,
                intrinsics=self.params['projection'],
                depth_image=depth_img
            )

            obj.position.x = x3d
            obj.position.y = y3d
            obj.position.z = z3d

            self.semantic_memory.insert_object(obj, tracked_msg.frame_id, timestamp)

        if self.frame_count % self.params['visualization']['frame_interval'] == 0:
            self.get_logger().info(f"💾 Saving scene graph at frame {self.frame_count}")
            graph = self.scene_graph_builder.build(self.semantic_memory)
            self.append_graph_to_jsonl(
                graph,
                frame_id=self.frame_count,
                save_path="/home/jack/ros2_ws/scene_graphs/scene_graph_history.jsonl",
                logger=self.get_logger()
            )

        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()