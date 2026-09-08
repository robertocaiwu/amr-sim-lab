# Copyright 2026 amr-sim-lab contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 2 node: gate /cmd_vel_raw -> /cmd_vel using a forward lidar arc."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from safety_stop.gate import apply_gate, obstacle_in_front


class SafetyStopNode(Node):

    def __init__(self) -> None:
        super().__init__("safety_stop")
        self.declare_parameter("stop_distance", 0.6)
        self.declare_parameter("front_arc_deg", 90.0)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("scan_timeout_sec", 1.0)

        self._last_scan: LaserScan | None = None
        self._last_scan_time = self.get_clock().now()

        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel_raw", self._on_cmd, 10)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._last_scan_time = self.get_clock().now()

    def _scan_is_fresh(self) -> bool:
        if self._last_scan is None:
            return False
        age = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
        return age <= float(self.get_parameter("scan_timeout_sec").value)

    def _on_cmd(self, msg: Twist) -> None:
        out = Twist()
        out.angular.z = msg.angular.z
        out.linear.x = msg.linear.x

        if not self._scan_is_fresh():
            if bool(self.get_parameter("require_scan").value):
                self._pub.publish(Twist())  # fail-safe: full stop
                self.get_logger().warn("no fresh /scan; holding robot",
                                       throttle_duration_sec=2.0)
                return
            self._pub.publish(out)
            return

        scan = self._last_scan
        blocked = obstacle_in_front(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
            float(self.get_parameter("stop_distance").value),
            math.radians(float(self.get_parameter("front_arc_deg").value)),
        )
        lin, ang = apply_gate(msg.linear.x, msg.angular.z, blocked)
        out.linear.x, out.angular.z = lin, ang
        if blocked and msg.linear.x > 0.0:
            self.get_logger().warn("obstacle within stop distance; blocking forward",
                                   throttle_duration_sec=2.0)
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
