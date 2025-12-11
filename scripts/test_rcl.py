import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IsaacTalker(Node):
    def __init__(self):
        super().__init__('isaac_talker')
        self.pub = self.create_publisher(String, 'isaac_chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Isaac container! [{self.count}]'
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main():
    rclpy.init()
    node = IsaacTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

