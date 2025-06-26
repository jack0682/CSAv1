from pathlib import Path
from csa_utils.config_utils import load_ros2_params

def load_parameters(config_path: str, node_name: str = "yolo_tracker_node") -> dict:
    """
    테스트 겸용: 기존 인터페이스를 유지하되 내부는 CSA 표준 방식으로 통합.
    향후 전면 교체 전까지 임시 래퍼로 사용 가능.
    """
    config_path = Path(config_path).expanduser().resolve()
    params = load_ros2_params(str(config_path), node_name)

    # 최소 유효성 검증 (필요시 주석처리 가능)
    for section in ['model', 'tracking', 'input']:
        if section not in params:
            raise ValueError(f"[⚠️] Missing required config section: '{section}'")

    return params
