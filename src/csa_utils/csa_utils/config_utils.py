import yaml
import os

def load_ros2_params(yaml_path: str, node_name: str) -> dict:
    """
    ROS2 YAML 파라미터 파일에서 주어진 노드명의 ros__parameters 하위 딕셔너리를 반환합니다.

    Args:
        yaml_path (str): YAML 파일 경로
        node_name (str): 노드 이름 (YAML 최상위 키)

    Returns:
        dict: ros__parameters 아래의 파라미터 딕셔너리

    Raises:
        FileNotFoundError: 파일이 존재하지 않을 경우
        KeyError: yaml 구조가 예상과 다를 경우
    """
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"[ConfigUtils] 파일 경로 없음: {yaml_path}")

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    try:
        return data[node_name]['ros__parameters']
    except KeyError as e:
        raise KeyError(
            f"[ConfigUtils] '{node_name} -> ros__parameters' 경로를 찾을 수 없습니다. "
            f"파일: {yaml_path}, KeyError: {str(e)}"
        )