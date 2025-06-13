# semantic_memory.py

from collections import defaultdict, deque
import os
import json
from copy import deepcopy

class SemanticMemory:
    def __init__(self, config):
        self.buffer_size = config['frame_buffer']['max_size']
        self.memory = deque(maxlen=self.buffer_size)  # [(frame_id, timestamp, [TrackedObject])]
        self.index_map = {}  # frame_id → index in deque

    def insert_object(self, obj, frame_id, timestamp):
        if frame_id in self.index_map:
            idx = self.index_map[frame_id]
            self.memory[idx][2].append(deepcopy(obj))
        else:
            self.memory.append((frame_id, timestamp, [deepcopy(obj)]))
            self.index_map[frame_id] = len(self.memory) - 1

    def get_objects_at(self, frame_id):
        for fid, _, objs in self.memory:
            if fid == frame_id:
                return objs
        return []

    def get_recent_frames(self, n=5):
        return list(self.memory)[-n:]

    def get_all(self):
        return list(self.memory)

    def save(self, save_path, run_id=None, format='json'):
        """
        Save the current semantic memory to a file.
        format: 'json' or 'jsonl'
        """
        os.makedirs(save_path, exist_ok=True)

        if format == 'json':
            self._save_as_json(save_path, run_id)
        elif format == 'jsonl':
            self._save_as_jsonl(save_path, run_id)
        else:
            raise ValueError(f"Unsupported format: {format}")

    def _save_as_json(self, save_path, run_id):
        output = []
        for frame_id, timestamp, objs in self.memory:
            frame_data = {
                'frame_id': frame_id,
                'timestamp': timestamp.sec + timestamp.nanosec * 1e-9,
                'objects': [self.obj_to_dict(obj) for obj in objs]
            }
            output.append(frame_data)

        filename = f'semantic_memory_{run_id or "latest"}.json'
        full_path = os.path.join(save_path, filename)

        with open(full_path, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"[💾] Semantic memory saved to {full_path} (JSON)")

    def _save_as_jsonl(self, save_path, run_id):
        filename = f'semantic_memory_{run_id or "latest"}.jsonl'
        full_path = os.path.join(save_path, filename)

        with open(full_path, 'w') as f:
            for frame_id, timestamp, objs in self.memory:
                frame_data = {
                    'frame_id': frame_id,
                    'timestamp': timestamp.sec + timestamp.nanosec * 1e-9,
                    'objects': [self.obj_to_dict(obj) for obj in objs]
                }
                f.write(json.dumps(frame_data) + '\n')
        print(f"[💾] Semantic memory saved to {full_path} (JSONL)")

    def obj_to_dict(self, obj):
        return {
            'track_id': obj.track_id,
            'label': obj.label,
            'position': {
                'x': obj.position.x,
                'y': obj.position.y,
                'z': obj.position.z
            },
            'feature': obj.feature,
            'relations': obj.relations,
            'confidence': obj.confidence,
            'frame_id': obj.frame_id
        }
