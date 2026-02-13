#!/usr/bin/env python3
from collections import deque
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from assignment2_rt.msg import ObstacleInfo
from assignment2_rt.srv import SetThreshold, GetAvgVel


def direction_from_angle(angle_rad: float) -> str:
    if -math.pi / 6 <= angle_rad <= math.pi / 6:
        return "front"
    if angle_rad > math.pi / 6:
        return "left"
    return "right"


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")

        self.threshold = 1.0
        self.last_cmd_raw = Twist()
        self.last5 = deque(maxlen=5)

        self.closest_dist = float("inf")
        self.closest_angle = 0.0

        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel_raw", self.on_cmd_raw, 10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_info = self.create_publisher(ObstacleInfo, "/obstacle_info", 10)

        self.create_service(SetThreshold, "/set_threshold", self.on_set_threshold)
        self.create_service(GetAvgVel, "/get_last_velocity", self.on_get_avg)

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            "SafetyNode started. Subscribing: /scan, /cmd_vel_raw. Publishing: /cmd_vel, /obstacle_info"
        )

    def on_cmd_raw(self, msg: Twist):
        self.last_cmd_raw = msg
        self.last5.append(msg)

    def on_scan(self, scan: LaserScan):
        min_r = float("inf")
        min_i = -1

        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                if r < min_r:
                    min_r = r
                    min_i = i

        if min_i >= 0:
            self.closest_dist = min_r
            self.closest_angle = scan.angle_min + min_i * scan.angle_increment
        else:
            self.closest_dist = float("inf")
            self.closest_angle = 0.0

    def control_loop(self):
        safe = self.closest_dist >= self.threshold

        out = Twist()

        if safe:
            out = self.last_cmd_raw
        else:
            turning = abs(self.last_cmd_raw.angular.z) > 1e-3
            if turning:
                out.linear.x = 0.0
                out.angular.z = self.last_cmd_raw.angular.z
            else:
                out.linear.x = 0.0
                out.angular.z = 0.0

        self.pub_cmd.publish(out)

        info = ObstacleInfo()
        info.distance = float(self.closest_dist) if math.isfinite(self.closest_dist) else 999.0
        info.direction = direction_from_angle(self.closest_angle)
        info.threshold = float(self.threshold)
        self.pub_info.publish(info)

    def on_set_threshold(self, req: SetThreshold.Request, res: SetThreshold.Response):
        if req.threshold <= 0.0:
            res.success = False
            return res
        self.threshold = float(req.threshold)
        self.get_logger().info(f"Threshold updated to {self.threshold:.3f} m")
        res.success = True
        return res

    def on_get_avg(self, req: GetAvgVel.Request, res: GetAvgVel.Response):
        if not self.last5:
            res.avg_linear_x = 0.0
            res.avg_angular_z = 0.0
            return res

        lin = sum(m.linear.x for m in self.last5) / len(self.last5)
        ang = sum(m.angular.z for m in self.last5) / len(self.last5)
        res.avg_linear_x = float(lin)
        res.avg_angular_z = float(ang)
        return res


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
