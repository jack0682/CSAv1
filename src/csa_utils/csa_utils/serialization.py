import json
import numpy as np
from geometry_msgs.msg import Point
from csa_interfaces.msg import TrackedObject
from datetime import datetime
from typing import Any, Dict, List, Type

# ===============================
# 🔹 기본 유틸
# ===============================

def serialize_point(point: Point) -> Dict[str, float]:
    return {'x': point.x, 'y': point.y, 'z': point.z}

def deserialize_point(data: Dict[str, float]) -> Point:
    point = Point()
    point.x = float(data.get('x', 0.0))
    point.y = float(data.get('y', 0.0))
    point.z = float(data.get('z', 0.0))
    return point

def serialize_numpy_array(arr: np.ndarray) -> List[float]:
    return arr.tolist()

def deserialize_numpy_array(data: List[float]) -> np.ndarray:
    return np.array(data, dtype=np.float32)

def serialize_datetime(dt: datetime) -> str:
    return dt.isoformat()

def deserialize_datetime(s: str) -> datetime:
    return datetime.fromisoformat(s)

# ===============================
# 🔹 TrackedObject
# ===============================

def serialize_tracked_object(obj: TrackedObject) -> Dict[str, Any]:
    return {
        'track_id': obj.track_id,
        'label': obj.label,
        'position': serialize_point(obj.position),
        'feature': list(obj.feature),
        'relations': list(obj.relations),
    }

def deserialize_tracked_object(data: Dict[str, Any]) -> TrackedObject:
    obj = TrackedObject()
    obj.track_id = int(data['track_id'])
    obj.label = str(data['label'])
    obj.position = deserialize_point(data['position'])
    obj.feature = [float(f) for f in data.get('feature', [])]
    obj.relations = [str(r) for r in data.get('relations', [])]
    return obj

# ===============================
# 🔹 SceneRelation (Custom)
# ===============================

def serialize_scene_relation(relation: Dict[str, Any]) -> Dict[str, Any]:
    return {
        'subject': relation['subject'],
        'relation': relation['relation'],
        'object': relation['object']
    }

def deserialize_scene_relation(data: Dict[str, Any]) -> Dict[str, str]:
    return {
        'subject': str(data['subject']),
        'relation': str(data['relation']),
        'object': str(data['object']),
    }

# ===============================
# 🔹 SceneGraphNode (Custom)
# ===============================

def serialize_scene_graph_node(node: Dict[str, Any]) -> Dict[str, Any]:
    return {
        'object': serialize_tracked_object(node['object']),
        'relations': [serialize_scene_relation(r) for r in node.get('relations', [])],
        'timestamp': serialize_datetime(node.get('timestamp', datetime.utcnow()))
    }

def deserialize_scene_graph_node(data: Dict[str, Any]) -> Dict[str, Any]:
    return {
        'object': deserialize_tracked_object(data['object']),
        'relations': [deserialize_scene_relation(r) for r in data.get('relations', [])],
        'timestamp': deserialize_datetime(data['timestamp'])
    }

# ===============================
# 🔹 JSON 저장/로드
# ===============================

def save_json(filepath: str, data: Any):
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)

def load_json(filepath: str) -> Any:
    with open(filepath, 'r') as f:
        return json.load(f)

def append_jsonl(filepath: str, data: Dict[str, Any]):
    with open(filepath, 'a') as f:
        json.dump(data, f)
        f.write('\n')

def load_jsonl(filepath: str) -> List[Dict[str, Any]]:
    with open(filepath, 'r') as f:
        return [json.loads(line) for line in f if line.strip()]

# ===============================
# 🔹 직렬화 라우팅 시스템 (Optional)
# ===============================

class SerializerRegistry:
    def __init__(self):
        self._serializers = {}
        self._deserializers = {}

    def register(self, type_name: str, serializer_fn, deserializer_fn):
        self._serializers[type_name] = serializer_fn
        self._deserializers[type_name] = deserializer_fn

    def serialize(self, type_name: str, obj: Any) -> Dict[str, Any]:
        return self._serializers[type_name](obj)

    def deserialize(self, type_name: str, data: Dict[str, Any]) -> Any:
        return self._deserializers[type_name](data)

# 전역 인스턴스
serializer_registry = SerializerRegistry()
serializer_registry.register('TrackedObject', serialize_tracked_object, deserialize_tracked_object)
serializer_registry.register('SceneRelation', serialize_scene_relation, deserialize_scene_relation)
serializer_registry.register('SceneGraphNode', serialize_scene_graph_node, deserialize_scene_graph_node)
