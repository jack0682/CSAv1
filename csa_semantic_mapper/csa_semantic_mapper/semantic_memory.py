from collections import deque
import os
import json
from copy import deepcopy
from rclpy.time import Time

from csa_utils.id_manager import RunIDManager
from csa_utils.rules.semanticobject import SemanticObject


class SemanticMemory:
    def __init__(self, config):
        self.buffer_size = config['frame_buffer']['max_size']
        self.memory = deque(maxlen=self.buffer_size)  # [(frame_id, timestamp, [SemanticObject])]
        self.index_map = {}  # frame_id → index in list(self.memory)
        self.run_id = RunIDManager().get()

    def insert_object(self, obj: SemanticObject, frame_id: int, timestamp: Time):
        """
        Insert a SemanticObject into the memory.
        If frame_id already exists, append to the object list of that frame.
        """
        found = False
        for i, (fid, ts, objs) in enumerate(self.memory):
            if fid == frame_id:
                objs.append(deepcopy(obj))
                found = True
                break
        if not found:
            self.memory.append((frame_id, timestamp, [deepcopy(obj)]))

    def get_objects_at(self, frame_id: int):
        for fid, _, objs in self.memory:
            if fid == frame_id:
                return objs
        return []

    def get_recent_frames(self, n=5):
        return list(self.memory)[-n:]

    def get_all(self):
        return list(self.memory)

    def save(self, save_root, format='json'):
        """
        Save the current semantic memory to a file.
        format: 'json' or 'jsonl'
        """
        save_path = os.path.join(save_root, self.run_id)
        os.makedirs(save_path, exist_ok=True)

        if format == 'json':
            self._save_as_json(save_path)
        elif format == 'jsonl':
            self._save_as_jsonl(save_path)
        else:
            raise ValueError(f"Unsupported format: {format}")

    def _save_as_json(self, save_path):
        output = []
        for frame_id, timestamp, objs in self.memory:
            frame_data = {
                'frame_id': frame_id,
                'timestamp': self._to_float_time(timestamp),
                'objects': [obj.to_dict() for obj in objs]
            }
            output.append(frame_data)

        full_path = os.path.join(save_path, f'semantic_memory_{self.run_id}.json')
        with open(full_path, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"[📂] Semantic memory saved to {full_path} (JSON)")

    def _save_as_jsonl(self, save_path):
        full_path = os.path.join(save_path, f'semantic_memory_{self.run_id}.jsonl')
        with open(full_path, 'w') as f:
            for frame_id, timestamp, objs in self.memory:
                frame_data = {
                    'frame_id': frame_id,
                    'timestamp': self._to_float_time(timestamp),
                    'objects': [obj.to_dict() for obj in objs]
                }
                f.write(json.dumps(frame_data) + '\n')
        print(f"[📂] Semantic memory saved to {full_path} (JSONL)")

    def _to_float_time(self, t):
        return t.sec + t.nanosec * 1e-9 if hasattr(t, 'sec') else float(t)
