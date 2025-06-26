# ===============================
# CSA: Semantic Tracked Object
# ===============================

# --- Semantic Identification ---
string label             # e.g., "chair", "cup"
int32 class_id           # e.g., 56
float32 confidence       # detection score (0~1)

# --- Spatial (3D Semantic Positioning) ---
geometry_msgs/Point position     # semantic 3D position in world coords
geometry_msgs/Vector3 velocity   # estimated linear velocity (vx, vy, vz)

# --- 2D Image Coordinates ---
int32 x_min
int32 y_min
int32 x_max
int32 y_max

# --- Identification and Provenance ---
int32 track_id           # Tracker ID
int32 frame_id           # Frame where detected
builtin_interfaces/Time stamp  # synchronized timestamp

# --- Embedding Features (for Re-ID, Clustering) ---
float32[128] feature     # 128D feature vector (optional)

# --- Ontological/Relational Slot (for Stage 2-5 Expansion) ---
string[] relations       # e.g., ["on(table)", "left-of(human)"]
string semantic_state    # optional: "active", "occluded", "dormant", etc.
string object_type       # e.g., "movable", "tool", "furniture" (optional)