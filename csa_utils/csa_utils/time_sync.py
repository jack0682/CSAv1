# time_sync.py

from builtin_interfaces.msg import Time
from rclpy.node import Node

def ros_time_to_float(t: Time) -> float:
    """Convert ROS2 Time to float (seconds)."""
    return t.sec + t.nanosec * 1e-9

def time_diff_sec(t1: Time, t2: Time) -> float:
    """Compute absolute difference in seconds between two ROS2 Time."""
    return abs(ros_time_to_float(t1) - ros_time_to_float(t2))

def is_within_slop(t1: Time, t2: Time, slop: float = 0.1) -> bool:
    """Return True if t1 and t2 are within `slop` seconds of each other."""
    return time_diff_sec(t1, t2) <= slop

def ros_now(node: Node) -> Time:
    """Return current ROS2 time from a given Node."""
    return node.get_clock().now().to_msg()
