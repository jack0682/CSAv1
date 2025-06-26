# csa_utils/rules/default_rules.py

from .base_rule import Rule
from . import rule_utils as ru

# 그룹별로 정의
proximity_rules = [

    Rule(
        name="very_close",
        priority=1,
        rule_fn=lambda o1, o2, thresholds: ru.euclidean_distance(o1.position, o2.position) < thresholds.get("very_close", 0.2),
        description="두 객체가 매우 가까운 거리 (예: 접촉 또는 거의 접촉)"
    ),

    Rule(
        name="close",
        priority=2,
        rule_fn=lambda o1, o2, thresholds: (
            0.2 <= ru.euclidean_distance(o1.position, o2.position) < thresholds.get("close", 0.5)
        ),
        description="두 객체가 가까움"
    ),

    Rule(
        name="far",
        priority=3,
        rule_fn=lambda o1, o2, thresholds: (
            ru.euclidean_distance(o1.position, o2.position) >= thresholds.get("far", 1.0)
        ),
        description="두 객체가 멀리 떨어져 있음"
    ),
]

spatial_rules = [

    Rule(
        name="on_top_of",
        priority=4,
        rule_fn=lambda o1, o2, thresholds: (
            ru.vertical_relation(o1.position, o2.position, threshold=thresholds.get("on", 0.2)) == "on"
            and ru.projected_overlap(o1.bbox, o2.bbox) > thresholds.get("overlap_ratio", 0.6)
        ),
        description="객체가 다른 객체 위에 있으며, 수평면상으로도 겹쳐 있음"
    ),

    Rule(
        name="under",
        priority=5,
        rule_fn=lambda o1, o2, thresholds: (
            ru.vertical_relation(o1.position, o2.position, threshold=thresholds.get("under", 0.2)) == "under"
        ),
        description="객체가 다른 객체 아래에 위치함"
    ),

    Rule(
        name="left_of",
        priority=6,
        rule_fn=lambda o1, o2, thresholds: (
            ru.horizontal_relation(o1.position, o2.position, axis="x", direction="left", threshold=thresholds.get("side", 0.2))
        ),
        description="객체가 다른 객체의 왼쪽에 위치함"
    ),

    Rule(
        name="right_of",
        priority=7,
        rule_fn=lambda o1, o2, thresholds: (
            ru.horizontal_relation(o1.position, o2.position, axis="x", direction="right", threshold=thresholds.get("side", 0.2))
        ),
        description="객체가 오른쪽에 위치함"
    ),

    Rule(
        name="in_front_of",
        priority=8,
        rule_fn=lambda o1, o2, thresholds: (
            ru.horizontal_relation(o1.position, o2.position, axis="y", direction="front", threshold=thresholds.get("front", 0.3))
        ),
        description="객체가 앞쪽에 위치함 (관찰자 기준)"
    ),

    Rule(
        name="behind",
        priority=9,
        rule_fn=lambda o1, o2, thresholds: (
            ru.horizontal_relation(o1.position, o2.position, axis="y", direction="back", threshold=thresholds.get("back", 0.3))
        ),
        description="객체가 뒤쪽에 위치함"
    ),
]

containment_rules = [

    Rule(
        name="inside",
        priority=10,
        rule_fn=lambda o1, o2, thresholds: (
            ru.contains(o2.bbox, o1.bbox, margin=thresholds.get("containment_margin", 0.05))
        ),
        description="객체 o1이 o2 안에 포함됨 (공간적 포함)"
    ),

    Rule(
        name="overlaps",
        priority=11,
        rule_fn=lambda o1, o2, thresholds: (
            ru.projected_overlap(o1.bbox, o2.bbox) > thresholds.get("overlap_ratio", 0.3)
        ),
        description="두 객체가 일정 비율 이상 겹침"
    ),
]

semantic_rules = [

    Rule(
        name="is_sitting_on",
        priority=12,
        rule_fn=lambda o1, o2, thresholds: (
            o1.class_name in ["person", "cat", "dog"]
            and o2.class_name in ["chair", "sofa"]
            and ru.vertical_relation(o1.position, o2.position, 0.3) == "on"
            and ru.projected_overlap(o1.bbox, o2.bbox) > 0.5
        ),
        description="의미적으로 사람이 의자 위에 앉아 있는 상태"
    ),

    Rule(
        name="is_in_container",
        priority=13,
        rule_fn=lambda o1, o2, thresholds: (
            o2.class_name in ["box", "drawer", "bag", "cup"]
            and ru.contains(o2.bbox, o1.bbox)
        ),
        description="객체가 박스/가방/컵 안에 들어있는 상태"
    ),
]

# 모든 규칙 합치기
DEFAULT_RULES = (
    proximity_rules +
    spatial_rules +
    containment_rules +
    semantic_rules
)
