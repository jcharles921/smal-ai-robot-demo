#!/usr/bin/env python3
"""
explorer_node - drives a linorobot2 (or any robot with /scan + /cmd_vel)
around autonomously using a simple reactive "gap following" strategy.

  subscribes : /scan     (sensor_msgs/LaserScan)
  publishes  : /cmd_vel  (geometry_msgs/Twist)
               /explorer/state (std_msgs/String)  - what the robot is doing

Run it while SLAM Toolbox is running and you get a map of the world without
touching the keyboard.

Parameters (see config/explorer.yaml):
  max_speed, max_turn, stop_dist, slow_dist, side_dist  - tuning
  enabled        - set false to pause:  ros2 param set /explorer enabled false
  max_runtime_s  - stop by itself after this many seconds (0 = never)
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np

from home_explorer.controller import ExplorerController


class ExplorerNode(Node):

    def __init__(self):
        super().__init__("explorer")
        p = self.declare_parameter
        p("max_speed", 0.25)
        p("max_turn", 1.0)
        p("stop_dist", 0.6)
        p("slow_dist", 1.3)
        p("side_dist", 0.32)
        p("rate_hz", 10.0)
        p("scan_timeout_s", 0.5)
        p("enabled", True)
        p("max_runtime_s", 0.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.ctrl = ExplorerController(
            max_speed=g("max_speed"), max_turn=g("max_turn"), stop_dist=g("stop_dist"),
            slow_dist=g("slow_dist"), side_dist=g("side_dist"))

        self.scan = None
        self.scan_time = None
        self.start_time = self.get_clock().now()
        self.last_state = ""

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "explorer/state", 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / g("rate_hz"), self.on_timer)
        self.get_logger().info("Explorer started - waiting for /scan ...")

    def on_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        # readings outside the sensor's valid band are treated as "no return"
        bad = (ranges < msg.range_min) | (ranges > msg.range_max)
        ranges[bad] = np.inf
        if self.scan is None:
            self.get_logger().info(
                f"Got first scan: {len(ranges)} beams, "
                f"{np.degrees(msg.angle_min):.0f}..{np.degrees(msg.angle_max):.0f} deg, "
                f"frame '{msg.header.frame_id}'")
        self.scan = (ranges, angles, float(msg.range_max))
        self.scan_time = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        cmd = Twist()
        state = "waiting_for_scan"

        runtime = self.get_parameter("max_runtime_s").value
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if not self.get_parameter("enabled").value:
            state = "paused"
        elif runtime > 0 and elapsed > runtime:
            state = "finished"
        elif self.scan is not None:
            age = (now - self.scan_time).nanoseconds * 1e-9
            if age > self.get_parameter("scan_timeout_s").value:
                state = "scan_timeout"          # safety: stale data -> stop
            else:
                ranges, angles, rmax = self.scan
                v, w, state = self.ctrl.compute(ranges, angles, far=rmax)
                cmd.linear.x, cmd.angular.z = v, w

        self.cmd_pub.publish(cmd)
        if state != self.last_state:
            self.get_logger().info(f"state: {state}")
            self.last_state = state
        self.state_pub.publish(String(data=state))

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()   # leave the robot standing still
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
