from csa_utils.rules.rule_manager import RuleManager
from csa_utils.rules.default_rules import get_default_rules
from csa_interfaces.msg import TrackedObject

# RuleManager 초기화 및 룰 등록
rm = RuleManager()
for rule in get_default_rules():
    rm.add_rule(rule)

# 객체 간 관계 추론
relations = rm.infer_relations(obj1, obj2)
print(relations)

# 설명 형태로 출력
print(rm.explain_relations(obj1, obj2))
