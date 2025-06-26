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

# 현재 시간을 기준으로 타임스탬프 생성
def timestamp():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]

# 로그 초기화 함수
def log_node_init(node: Node):
    node.get_logger().info(Color.CYAN + f"🔧 [{timestamp()}] Node initialized" + Color.RESET)

# 성공 로그 출력
def log_success(node: Node, msg: str):
    node.get_logger().info(Color.GREEN + f"✅ [{timestamp()}] {msg}" + Color.RESET)

# 정보 로그 출력
def log_info(node: Node, msg: str):
    node.get_logger().info(Color.GREEN + f"✔️ [{timestamp()}] {msg}" + Color.RESET)

# 경고 로그 출력
def log_warn(node: Node, msg: str):
    node.get_logger().warn(Color.YELLOW + f"⚠️ [{timestamp()}] {msg}" + Color.RESET)

# 에러 로그 출력
def log_error(node: Node, msg: str):
    node.get_logger().error(Color.RED + f"❌ [{timestamp()}] {msg}" + Color.RESET)

# 예외 처리 및 에러 로그 출력 (특별한 예외 메시지 포함)
def log_exception(node: Node, msg: str, exception: Exception):
    node.get_logger().error(Color.RED + f"❌ [{timestamp()}] {msg}: {exception}" + Color.RESET)
