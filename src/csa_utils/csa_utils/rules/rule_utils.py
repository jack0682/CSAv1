import numpy as np
from typing import Tuple, Dict
from csa_utils.rules.semanticobject import SemanticObject


# --- 거리 및 위치 정보 계산 함수 ---

def euclidean_distance(obj1: SemanticObject, obj2: SemanticObject) -> float:
    return np.linalg.norm(obj1.position - obj2.position)


def height_difference(obj1: SemanticObject, obj2: SemanticObject) -> float:
    return obj1.position[2] - obj2.position[2]


def horizontal_difference(obj1: SemanticObject, obj2: SemanticObject) -> float:
    return np.linalg.norm(obj1.position[:2] - obj2.position[:2])


def directional_relation(obj1: SemanticObject, obj2: SemanticObject) -> str:
    dx, dy = obj2.position[0] - obj1.position[0], obj2.position[1] - obj1.position[1]
    angle = np.arctan2(dy, dx) * 180 / np.pi

    if -45 <= angle < 45:
        return "right_of"
    elif 45 <= angle < 135:
        return "front_of"
    elif angle >= 135 or angle < -135:
        return "left_of"
    else:
        return "behind_of"


# --- 정렬 및 상하관계 판단 ---

def is_on_top(obj1: SemanticObject, obj2: SemanticObject, z_threshold: float = 0.1) -> bool:
    dz = obj1.position[2] - obj2.position[2]
    return 0 < dz < z_threshold


def is_aligned_horizontally(obj1: SemanticObject, obj2: SemanticObject, tolerance: float = 0.05) -> bool:
    return abs(obj1.position[1] - obj2.position[1]) < tolerance


def is_aligned_vertically(obj1: SemanticObject, obj2: SemanticObject, tolerance: float = 0.05) -> bool:
    return np.linalg.norm(obj1.position[:2] - obj2.position[:2]) < tolerance


# --- 관계 명시적 판단 ---

def is_near(obj1: SemanticObject, obj2: SemanticObject, threshold: float = 0.5) -> bool:
    return euclidean_distance(obj1, obj2) < threshold


def is_under(obj1: SemanticObject, obj2: SemanticObject, z_thresh: float = 0.15) -> bool:
    return obj1.position[2] < obj2.position[2] - z_thresh


def is_above(obj1: SemanticObject, obj2: SemanticObject, z_thresh: float = 0.15) -> bool:
    return obj1.position[2] > obj2.position[2] + z_thresh


# --- 구조화된 특징 추출 ---

def get_spatial_features(obj1: SemanticObject, obj2: SemanticObject) -> Dict:
    return {
        "distance": euclidean_distance(obj1, obj2),
        "height_diff": height_difference(obj1, obj2),
        "horizontal_diff": horizontal_difference(obj1, obj2),
        "direction": directional_relation(obj1, obj2),
        "aligned_horizontally": is_aligned_horizontally(obj1, obj2),
        "aligned_vertically": is_aligned_vertically(obj1, obj2),
        "is_on_top": is_on_top(obj1, obj2),
        "is_near": is_near(obj1, obj2),
        "is_above": is_above(obj1, obj2),
        "is_under": is_under(obj1, obj2),
    }
