from abc import ABC, abstractmethod
from typing import Tuple, Dict, Any

class SemanticObject(ABC):
    """
    CSA 시스템의 모든 의미 객체의 추상 기반 클래스.
    TrackedObject, ToolObject, HumanIntentObject 등의 상위 개념으로 사용됨.
    모든 의미 객체는 고유 ID, 타입, 위치, 직렬화 기능을 제공해야 함.
    """

    @abstractmethod
    def get_id(self) -> str:
        """객체 고유 ID 반환"""
        pass

    @abstractmethod
    def get_type(self) -> str:
        """객체 타입 반환"""
        pass

    @abstractmethod
    def get_position(self) -> Tuple[float, float, float]:
        """객체의 3D 위치 반환"""
        pass

    @abstractmethod
    def to_dict(self) -> Dict[str, Any]:
        """객체의 직렬화된 dictionary 형태 반환"""
        pass

    def __repr__(self):
        return f"{self.get_type()}(id={self.get_id()}, pos={self.get_position()})"
