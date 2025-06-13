# scene_graph_builder.py

import os
import json
import networkx as nx
import numpy as np

class SceneGraphBuilder:
    def __init__(self, config):
        self.thresholds = config['relation_thresholds']
        self.output_path = config['output']['save_path']
        self.output_format = config['output']['save_format']
        os.makedirs(self.output_path, exist_ok=True)

    def build(self, semantic_memory):
        latest_frame = semantic_memory.get_recent_frames(1)[0]
        frame_id, timestamp, objects = latest_frame

        G = nx.DiGraph()
        for obj in objects:
            G.add_node(obj.track_id, label=obj.label, x=obj.position.x, y=obj.position.y, z=obj.position.z)

        for i in range(len(objects)):
            for j in range(len(objects)):
                if i == j:
                    continue
                rel = self.infer_relations(objects[i], objects[j])
                if rel:
                    G.add_edge(objects[i].track_id, objects[j].track_id, relation=rel)

        return G

    def infer_relations(self, obj1, obj2):
        p1 = np.array([obj1.position.x, obj1.position.y, obj1.position.z])
        p2 = np.array([obj2.position.x, obj2.position.y, obj2.position.z])
        dist = np.linalg.norm(p1 - p2)
        dz = p2[2] - p1[2]  # height diff

        # Relation rules
        if dist < self.thresholds['near']:
            return "near"
        elif abs(dz) < self.thresholds['on'] and p1[2] < p2[2]:
            return "on"
        elif dz > self.thresholds['above']:
            return "above"
        return None

    def save(self, graph, frame_id):
        if self.output_format == "json":
            data = nx.readwrite.json_graph.node_link_data(graph)
            out_path = os.path.join(self.output_path, f"scene_graph_{frame_id}.json")
            with open(out_path, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"[💾] Scene Graph (JSON) saved to: {out_path}")

        elif self.output_format == "graphml":
            out_path = os.path.join(self.output_path, f"scene_graph_{frame_id}.graphml")
            nx.write_graphml(graph, out_path)
            print(f"[💾] Scene Graph (GraphML) saved to: {out_path}")

        elif self.output_format == "jsonl":
            data = nx.readwrite.json_graph.node_link_data(graph)
            out_path = os.path.join(self.output_path, f"scene_graph_history.jsonl")
            with open(out_path, 'a') as f:
                json.dump(data, f)
                f.write('\n')
            print(f"[💾] Scene Graph (JSONL) appended to: {out_path}")
            
            
