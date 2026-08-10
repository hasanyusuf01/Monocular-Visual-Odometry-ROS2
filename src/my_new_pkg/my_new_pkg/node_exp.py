import rclpy
from rclpy.node import Node

class CustomeNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info('hey!')

def main(args=None):
    rclpy.init(args=args)
    node = CustomeNode('exmp1')
    # node = Node('exmp')
    # node.get_logger().info('hey!')
    rclpy.spin(node)
    rclpy.shutdown()
    # pass

if __name__ == "__main__":
    main()