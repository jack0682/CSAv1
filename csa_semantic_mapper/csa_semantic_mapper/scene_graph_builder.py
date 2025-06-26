import os
import json
import networkx as nx
from csa_utils.rules.rule_manager import RuleManager
from csa_utils.rules.default_rules import get_default_rules
from csa_interfaces.msg import TrackedObject
from geometry_msgs.msg import Point, Vector3

class SceneGraphBuilder:
    def __init__(self, config):
        """
        SceneGraphBuilder 클래스 초기화

        Args:
            config (dict): 설정 정보를 담은 딕셔너리
                - rule_mode: 'default' 또는 'custom'
                - custom_rules: Rule 객체 리스트 (선택사항)
                - output: {
                    save_path (str): 저장 경로
                    save_format (str): json / jsonl / graphml
                }
        """
        self.rule_manager = self._init_rule_manager(config)
        self.output_path = config['output']['save_path']
        self.output_format = config['output']['save_format']
        os.makedirs(self.output_path, exist_ok=True)

    def _init_rule_manager(self, config):
        """
        RuleManager 초기화: 기본 규칙 또는 사용자 정의 규칙을 사용하여 초기화합니다.
        """
        if config.get("rule_mode", "default") == "custom" and "custom_rules" in config:
            return RuleManager(config['custom_rules'])
        else:
            return RuleManager(get_default_rules())

    def build(self, semantic_memory):
        """
        최신 semantic_memory로부터 Scene Graph를 생성합니다.

        Args:
            semantic_memory (SemanticMemory): 최근 프레임 객체 상태를 가진 메모리 객체

        Returns:
            networkx.DiGraph: 생성된 scene graph 객체
        """
        try:
            latest_frame = semantic_memory.get_recent_frames(1)[0]
        except Exception as e:
            raise RuntimeError(f"[SceneGraphBuilder] 최근 프레임 데이터를 가져올 수 없습니다: {e}")

        frame_id, timestamp, objects = latest_frame
        G = nx.DiGraph()

        # 노드 추가: 각 객체를 Graph의 노드로 추가
        for obj in objects:
            # TrackedObject 객체 초기화
            tracked_obj = TrackedObject()
            tracked_obj.track_id = obj.track_id
            tracked_obj.label = obj.label
            tracked_obj.confidence = obj.confidence
            tracked_obj.position = Point(x=obj.position.x, y=obj.position.y, z=obj.position.z)
            tracked_obj.velocity = Vector3(x=obj.velocity.x, y=obj.velocity.y, z=obj.velocity.z)

            # 객체를 그래프의 노드로 추가
            G.add_node(
                tracked_obj.track_id,
                label=tracked_obj.label,
                x=tracked_obj.position.x,
                y=tracked_obj.position.y,
                z=tracked_obj.position.z
            )

        # 객체 간 관계 추론 및 엣지 추가
        for i in range(len(objects)):
            for j in range(len(objects)):
                if i == j:
                    continue
                # 객체 간 관계 추론
                relations = self.rule_manager.infer_relations(objects[i], objects[j])
                for rel in relations:
                    # 추론된 관계를 엣지로 추가
                    G.add_edge(objects[i].track_id, objects[j].track_id, relation=rel)

        return G

    def save(self, graph, frame_id):
        """
        생성된 Scene Graph를 저장합니다.

        Args:
            graph (nx.DiGraph): 저장할 그래프
            frame_id (int): 해당 그래프가 속한 프레임 번호
        """
        file_prefix = f"scene_graph_{frame_id}"

        # JSON 형식으로 저장
        if self.output_format == "json":
            out_path = os.path.join(self.output_path, f"{file_prefix}.json")
            data = nx.readwrite.json_graph.node_link_data(graph)
            with open(out_path, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"[💾] Scene Graph (JSON) saved to: {out_path}")

        # GraphML 형식으로 저장
        elif self.output_format == "graphml":
            out_path = os.path.join(self.output_path, f"{file_prefix}.graphml")
            nx.write_graphml(graph, out_path)
            print(f"[💾] Scene Graph (GraphML) saved to: {out_path}")

        # JSONL 형식으로 저장
        elif self.output_format == "jsonl":
            out_path = os.path.join(self.output_path, "scene_graph_history.jsonl")
            data = nx.readwrite.json_graph.node_link_data(graph)
            with open(out_path, 'a') as f:
                json.dump(data, f)
                f.write('\n')
            print(f"[💾] Scene Graph (JSONL) appended to: {out_path}")

        else:
            raise ValueError(f"[SceneGraphBuilder] 지원되지 않는 저장 형식입니다: {self.output_format}")
