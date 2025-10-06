#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(String, 'shape_topic', 10)
        self.timer = self.create_timer(2.0, self.get_shape)

    def get_shape(self):
        shape = input("Enter a shape:\n-heart,flower,star\n\n-clear,stop\n").strip().lower()
        msg = String()
        msg.data = shape
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
