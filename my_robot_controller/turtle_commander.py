#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import math
import time
import threading

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')
        self.subscriber = self.create_subscription(String, 'shape_topic', self.shape_callback, 10)

        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for teleport service...")

        self.clear_cli = self.create_client(Empty, '/clear')
        while not self.clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for clear service...")

        self.current_shape = None
        self.stop = False
        self.draw_thread = None

    def shape_callback(self, msg):
        shape = msg.data.strip().lower()

        if shape == "stop":
            self.stop = True
            self.get_logger().info("Stopped drawing.")
            return

        if shape == "clear":
            self.stop = True  
            if self.draw_thread:
                self.draw_thread.join()
            self.clear_screen()
            self.reset_position()
            self.clear_screen()
            self.get_logger().info("Screen cleared and turtle reset.")
            return

        self.stop = False
        self.current_shape = shape

        if self.draw_thread and self.draw_thread.is_alive():
            self.stop = True
            self.draw_thread.join()

        self.stop = False
        self.draw_thread = threading.Thread(target=self.draw_shape, args=(shape,))
        self.draw_thread.start()

    def teleport(self, x, y, theta=0.0):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self.teleport_cli.call_async(req)

    def clear_screen(self):
        req = Empty.Request()
        self.clear_cli.call_async(req)
        time.sleep(0.5)

    def reset_position(self):
        self.teleport(5.5, 5.5, 0.0)
        time.sleep(0.2)

    def draw_shape(self, shape):
        if shape == "heart":
            self.draw_heart()
        elif shape == "flower":
            self.draw_flower()
        elif shape == "star":
            self.draw_star()
        else:
            self.get_logger().warn("Unknown shape")

    def draw_heart(self):
        for t in [i * 0.05 for i in range(0, 630)]:
            if self.stop: break
            x = 5.5 + (16 * math.sin(t)**3) / 8
            y = 5.5 + (13*math.cos(t) - 5*math.cos(2*t) - 2*math.cos(3*t) - math.cos(4*t)) / 8
            self.teleport(x, y)
            time.sleep(0.01)

    def draw_flower(self):
        R = 4
        for t in [i * 0.01 for i in range(0, 628)]:
            if self.stop: break
            r = R * math.cos(5*t)
            x = 5.5 + r * math.cos(t) / 2
            y = 5.5 + r * math.sin(t) / 2
            self.teleport(x, y)
            time.sleep(0.01)

    def draw_star(self):
        R = 3  
        for t in [i * 0.01 for i in range(0, 628)]:  
            if self.stop: break
            r = R * (1 + 0.5 * math.sin(5 * t))  
            x = 5.5 + r * math.cos(t)
            y = 5.5 + r * math.sin(t)
            self.teleport(x, y)
            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
