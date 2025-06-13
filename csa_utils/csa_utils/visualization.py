# visualization.py

import matplotlib.pyplot as plt
import networkx as nx
import os

def draw_scene_graph(graph: nx.DiGraph, title="CSA Scene Graph", save_path=None, frame_id=None):
    """
    Visualize a directed scene graph using networkx + matplotlib.
    """
    plt.figure(figsize=(8, 6))
    pos = nx.spring_layout(graph, seed=42)

    node_labels = nx.get_node_attributes(graph, 'label')
    edge_labels = nx.get_edge_attributes(graph, 'relation')

    nx.draw_networkx_nodes(graph, pos, node_size=800, node_color='skyblue')
    nx.draw_networkx_edges(graph, pos, arrowstyle='->', arrowsize=15, width=2)
    nx.draw_networkx_labels(graph, pos, labels=node_labels, font_size=10, font_weight='bold')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_color='gray', font_size=8)

    plt.title(title + (f" (Frame {frame_id})" if frame_id else ""))
    plt.axis('off')

    if save_path:
        os.makedirs(save_path, exist_ok=True)
        file_name = f"scene_graph_{frame_id or 'latest'}.png"
        plt.savefig(os.path.join(save_path, file_name))
        print(f"[🖼️] Scene Graph saved to {os.path.join(save_path, file_name)}")
    else:
        plt.show()

    plt.close()
