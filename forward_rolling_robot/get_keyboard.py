import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import sys
import select
import termios
import tty

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('get_keyboard')
        self.publisher_ = self.create_publisher(Int16, 'cmd_vel', 3)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.key_value = Int16()
        self.settings = termios.tcgetattr(sys.stdin)
        self.target_key = None

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        dr, dw, de = select.select([sys.stdin], [], [], 0.1)  # Timeout 설정을 0.1초로 변경
        if dr:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.get_key()

        if key == 'w':
            self.key_value.data = 1  # forward
        elif key == 's':
            self.key_value.data = 2  # backward
        elif key == 'a':
            self.key_value.data = 3  # left
        elif key == 'd':
            self.key_value.data = 4  # right
        elif key == '\x03':  # Ctrl+C
            raise KeyboardInterrupt
        else:
            self.key_value.data = 0  # stop

        self.publisher_.publish(self.key_value)
        if key is not None:  # 키 입력이 있을 때만 로그 출력
            self.get_logger().info(f'Publishing: "{self.key_value.data}", Key: "{key}"')
        else:
            self.get_logger().info(f'Publishing: "{self.key_value.data}", Key: "None"')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

if __name__ == '__main__':
    main()
