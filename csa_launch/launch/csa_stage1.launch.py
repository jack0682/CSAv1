from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Config 파일 경로 정의
    slam_config = os.path.join(
        get_package_share_directory('csa_slam_interface'),
        'config', 'slam_interface.yaml'
    )
    yolo_config = os.path.join(
        get_package_share_directory('csa_yolo_inference'),
        'config', 'global_parameters.yaml'
    )
    mapper_config = os.path.join(
        get_package_share_directory('csa_semantic_mapper'),
        'config', 'semantic_mapper.yaml'
    )
    image_input_config = os.path.join(
        get_package_share_directory('csa_image_input'),
        'config', 'image_input.yaml'
    )

     # 🎥 이미지 퍼블리셔 노드 (Timer로 10초 후 실행)
    image_publisher_node = TimerAction(
        period=10.0,  # 10초 지연
        actions=[
            Node(
                package='csa_image_input',
                executable='image_publisher_node',
                name='image_publisher_node',
                output='screen',
                parameters=[image_input_config]
            )
        ]
    )

    return LaunchDescription([
        # 🛰️ 1. SLAM Pose 중계
        Node(
            package='csa_slam_interface',
            executable='slam_pose_node',
            name='slam_pose_node',
            output='screen',
            parameters=[slam_config]
        ),

        # 🔍 2. YOLO 추론 + 객체 추적
        Node(
            package='csa_yolo_inference',
            executable='yolo_tracker_node',
            name='yolo_tracker_node',
            output='screen',
            parameters=[yolo_config]
        ),

        # 🧠 3. 의미 좌표 추론 및 Scene Graph 구축
        Node(
            package='csa_semantic_mapper',
            executable='semantic_mapper_node',
            name='semantic_mapper_node',
            output='screen',
            parameters=[mapper_config]
        ),

        # 🎥 4. 지연된 이미지 퍼블리셔
        image_publisher_node
    ])
