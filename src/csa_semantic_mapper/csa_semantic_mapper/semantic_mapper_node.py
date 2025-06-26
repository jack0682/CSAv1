import os
import json
import traceback
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

# 외부 모듈을 함수 내부로 이동시킴
class SemanticMapperNode(Node):
    def __init__(self):
        super().__init__('semantic_mapper_node')

        # Log initialization moved inside to avoid circular imports
        from csa_utils.log import log_node_init, log_success, log_error
        log_node_init(self, self.get_name())

        # Load parameters
        self.params = self._load_config()
        self.visualization_interval = self.params['visualization'].get('frame_interval', 10)
        self.enable_reasoning = self.params.get('reasoning', {}).get('enabled', True)

        # Core modules
        self.semantic_memory = SemanticMemory(self.params)
        self.scene_graph_builder = SceneGraphBuilder(self.params)
        self.rule_manager = RuleManager(self.params) if self.enable_reasoning else None
        self.bridge = CvBridge()

        # ROS interface
        self.cb_group = ReentrantCallbackGroup()
        self._setup_subscribers()

        self.frame_count = 0
        log_success(self, "🟢 SemanticMapperNode ready. Waiting for synchronized messages...")

    def _load_config(self):
        config_path = Path(__file__).parent.parent / 'config/semantic_mapper.yaml'
        try:
            return load_ros2_params(str(config_path), self.get_name())
        except Exception as e:
            from csa_utils.log import log_error
            log_error(self, f"❌ Error loading config: {e}")
            return {}

    def _setup_subscribers(self):
        self.sub_tracked = Subscriber(
            self, TrackedObjectArray, '/tracked_objects', qos_profile_sensor_data,
            callback_group=self.cb_group)
        self.sub_pose = Subscriber(
            self, PoseStamped, '/camera/pose', qos_profile_sensor_data,
            callback_group=self.cb_group)
        self.sub_depth = Subscriber(
            self, Image, '/camera/depth/image_raw', qos_profile_sensor_data,
            callback_group=self.cb_group)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_tracked, self.sub_pose, self.sub_depth],
            queue_size=30, slop=2.0, allow_headerless=True)
        self.sync.registerCallback(self.synced_callback)

    def synced_callback(self, tracked_msg, pose_msg, depth_msg):
        try:
            timestamp = tracked_msg.header.stamp
            camera_pose = pose_msg.pose
            t_pose = ros_time_to_float(pose_msg.header.stamp)
            t_tracked = ros_time_to_float(tracked_msg.header.stamp)
            t_depth = ros_time_to_float(depth_msg.header.stamp)

            self.get_logger().info(
                f"📥 [SYNC] t_pose={t_pose:.3f}, t_tracked={t_tracked:.3f}, t_depth={t_depth:.3f}, Δt={abs(t_pose - t_tracked):.3f}")

            depth_img = self._convert_depth_image(depth_msg)
            if depth_img is None:
                return

            self._process_tracked_objects(tracked_msg, camera_pose, depth_img, timestamp)

            if self.enable_reasoning:
                self._run_reasoning()

            if self.frame_count % self.visualization_interval == 0:
                self._build_and_save_scene_graph()

            self.frame_count += 1

        except Exception as e:
            from csa_utils.log import log_error
            log_error(self, f"[❌ Exception in synced_callback]: {e}")
            self.get_logger().error(traceback.format_exc())

    def _convert_depth_image(self, depth_msg):
        try:
            return self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            from csa_utils.log import log_error
            log_error(self, f"❌ Depth image 변환 실패: {e}")
            return None

    def _process_tracked_objects(self, tracked_msg, camera_pose, depth_img, timestamp):
        for obj in tracked_msg.objects:
            try:
                x3d, y3d, z3d = project_bbox_to_3d(
                    bbox=(obj.x_min, obj.y_min, obj.x_max, obj.y_max),
                    camera_pose=camera_pose,
                    intrinsics=self.params['projection'],
                    depth_image=depth_img
                )
                obj.position.x, obj.position.y, obj.position.z = x3d, y3d, z3d
                self.semantic_memory.insert_object(obj, tracked_msg.frame_id, timestamp)
            except Exception as e:
                from csa_utils.log import log_error
                log_error(self, f"❗ 3D 위치 추정 실패: {e}")

    def _run_reasoning(self):
        try:
            self.rule_manager.run(self.semantic_memory)
        except Exception as e:
            from csa_utils.log import log_error
            log_error(self, f"❌ 추론 실패: {e}")

    def _build_and_save_scene_graph(self):
        try:
            graph = self.scene_graph_builder.build(self.semantic_memory)
            self.scene_graph_builder.save(graph, frame_id=self.frame_count)
            self.get_logger().info(f"💾 Scene graph 저장 완료 (frame={self.frame_count})")
        except Exception as e:
            from csa_utils.log import log_error
            log_error(self, f"❌ Scene graph 저장 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapperNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
