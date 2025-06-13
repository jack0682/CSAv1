# log.py

from rclpy.node import Node
from datetime import datetime

# ANSI Color Codes
class Color:
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    GRAY = "\033[90m"
    CYAN = "\033[96m"
    RESET = "\033[0m"

def timestamp():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]

def log_node_init(node: Node, name: str):
    msg = f"✅ [{timestamp()}] Node '{name}' initialized successfully."
    node.get_logger().info(Color.CYAN + msg + Color.RESET)

def log_success(node: Node, msg: str):
    node.get_logger().info(Color.GREEN + f"✔️ [{timestamp()}] " + msg + Color.RESET)

def log_warning(node: Node, msg: str):
    node.get_logger().warn(Color.YELLOW + f"⚠️ [{timestamp()}] " + msg + Color.RESET)

def log_error(node: Node, msg: str):
    node.get_logger().error(Color.RED + f"❌ [{timestamp()}] " + msg + Color.RESET)

def log_debug(node: Node, msg: str):
    node.get_logger().debug(Color.GRAY + f"🔍 [{timestamp()}] " + msg + Color.RESET)
