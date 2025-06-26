from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ⚙️ 공통 파라미터
    use_sim_time = {'use_sim_time': True}

    # 📁 패키지별 config 디렉토리
    slam_pkg = get_package_share_directory('csa_slam_interface')
    yolo_pkg = get_package_share_directory('csa_yolo_inference')
    mapper_pkg = get_package_share_directory('csa_semantic_mapper')
    image_input_pkg = get_package_share_directory('csa_image_input')

    # 📄 config 파일 경로
    yolo_config_path = os.path.join(yolo_pkg, 'config', 'global_parameters.yaml')
    mapper_config_path = os.path.join(mapper_pkg, 'config', 'semantic_mapper.yaml')
    image_input_config_path = os.path.join(image_input_pkg, 'config', 'image_input.yaml')

    # 📌 SLAM 노드 (C++ → 수동 래핑 → YAML 생략)
    slam_pose_node = Node(
        package='csa_slam_interface',
        executable='slam_pose_node',
        name='slam_pose_node',
        output='screen',
        parameters=[
            use_sim_time,
            {
                'vocab_path': '/home/jack/ORB_SLAM2/Vocabulary/ORBvoc.txt',
                'config_path': '/home/jack/ORB_SLAM2/Examples/RGB-D/TUM1.yaml',
                'frame_id': 'map',
                'max_time_diff': 0.5
            }
        ]
    )

    # 🟡 YOLO 추적기 노드
    yolo_tracker_node = Node(
        package='csa_yolo_inference',
        executable='yolo_tracker_node',
        name='yolo_tracker_node',
        output='screen',
        parameters=[
            yolo_config_path,
            use_sim_time
        ]
    )

    # 🟢 의미 매핑 노드
    semantic_mapper_node = Node(
        package='csa_semantic_mapper',
        executable='semantic_mapper_node',
        name='semantic_mapper_node',
        output='screen',
        parameters=[
            mapper_config_path,
            use_sim_time
        ]
    )

    # 🔵 이미지 퍼블리셔 노드 (10초 지연 실행)
    image_publisher_launch = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='csa_image_input',
                executable='image_publisher_node',
                name='image_publisher_node',
                output='screen',
                parameters=[
                    image_input_config_path,
                    use_sim_time
                ]
            )
        ]
    )

    # 📦 LaunchDescription 구성
    return LaunchDescription([
        slam_pose_node,
        yolo_tracker_node,
        semantic_mapper_node,
        image_publisher_launch
    ])
