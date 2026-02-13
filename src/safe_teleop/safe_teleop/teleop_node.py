#!/usr/bin/env python3
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
teleop_node (safe_teleop)
Publishes: /cmd_vel_raw (geometry_msgs/Twist)

Keys:
  w: forward
  s: backward
  a: turn left
  d: turn right
  space: stop
  q: quit
"""

def get_key(timeout=0.1):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.pub = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.lin = 0.2
        self.ang = 0.8

        self.get_logger().info(HELP.strip())
        self._settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.timer = self.create_timer(0.05, self._loop)

    def _loop(self):
        key = get_key()
        if key is None:
            return

        msg = Twist()

        if key == "w":
            msg.linear.x = self.lin
        elif key == "s":
            msg.linear.x = -self.lin
        elif key == "a":
            msg.angular.z = self.ang
        elif key == "d":
            msg.angular.z = -self.ang
        elif key == " ":
            pass
        elif key == "q":
            raise KeyboardInterrupt
        else:
            return

        self.pub.publish(msg)

    def destroy_node(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
