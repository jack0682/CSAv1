# ===============================
# CSA: Array of Semantic Tracked Objects
# ===============================

std_msgs/Header header                 # ROS 표준 헤더 (stamp + frame_id)

builtin_interfaces/Time header_stamp  # [Legacy] 개별 분석 용도
int32 frame_id                        # frame number (shared across all objects)

csa_interfaces/TrackedObject[] objects
