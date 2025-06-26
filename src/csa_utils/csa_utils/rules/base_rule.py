from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
from csa_interfaces.msg import TrackedObject

class RelationRule(ABC):
    """
    관계 추론을 위한 추상 규칙 클래스.
    모든 관계 룰은 이 클래스를 상속받아 구현됨.
    """

    def __init__(
        self,
        name: str,
        priority: int = 10,
        thresholds: Optional[Dict[str, float]] = None,
        metadata: Optional[Dict[str, Any]] = None
    ):
        """
        :param name: 규칙 이름 (예: 'near', 'above')
        :param priority: 적용 우선순위 (낮을수록 먼저 적용됨)
        :param thresholds: 비교를 위한 임계값 딕셔너리
        :param metadata: 규칙에 부가적인 정보를 담을 수 있는 사전
        """
        self.name = name
        self.priority = priority
        self.thresholds = thresholds or {}
        self.metadata = metadata or {}

    @abstractmethod
    def evaluate(self, obj1: TrackedObject, obj2: TrackedObject) -> bool:
        """
        두 객체 간의 관계를 판단하는 핵심 함수.
        True일 경우 해당 관계 성립으로 간주.
        """
        pass

    def __call__(self, obj1: TrackedObject, obj2: TrackedObject) -> bool:
        return self.evaluate(obj1, obj2)

    def __repr__(self):
        return f"<RelationRule(name='{self.name}', priority={self.priority})>"

    def get_relation_tuple(self, obj1: TrackedObject, obj2: TrackedObject) -> Optional[Dict[str, Any]]:
        """
        관계가 성립할 경우 반환할 관계 정보 딕셔너리 형식 (JSONL 저장 등 활용 가능)
        """
        if self.evaluate(obj1, obj2):
            return {
                "subject": obj1.label,
                "object": obj2.label,
                "relation": self.name,
                "priority": self.priority,
                "confidence": self.metadata.get("confidence", 1.0),  # 향후 확률 기반 추론과 연결 가능
            }
        return None

    def with_threshold(self, key: str) -> float:
        """
        임계값이 존재하지 않을 경우 기본값 0.0을 리턴
        """
        return self.thresholds.get(key, 0.0)

    def describe(self) -> str:
        """
        규칙의 간단한 설명 리턴 (디버깅 및 로깅용)
        """
        return f"Rule: {self.name}, Priority: {self.priority}, Thresholds: {self.thresholds}"
