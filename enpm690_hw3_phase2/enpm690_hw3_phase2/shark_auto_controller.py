#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .constants import LIDAR_BIN_COUNT, SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_LINEAR_SPEED, SharkState
from .geometry_utils import angle_diff, bearing_xy, clamp, distance_xy
from .observation_builder import ObservationBuilder
from .spawn_utils import fish_states_from_json, game_snapshot_from_json


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SharkAutoController(Node):
    def __init__(self) -> None:
        super().__init__("shark_auto_controller")
        self.declare_parameter("cmd_topic", "/cmd_vel_input")
        self.declare_parameter("k_target", 1.6)
        self.declare_parameter("k_avoid", 1.4)
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("turn_in_place_proximity", 0.72)
        self.declare_parameter("cruise_speed", 0.22)
        self.declare_parameter("max_speed_scale", 1.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.k_target = float(self.get_parameter("k_target").value)
        self.k_avoid = float(self.get_parameter("k_avoid").value)
        control_hz = float(self.get_parameter("control_hz").value)
        self.turn_in_place_proximity = float(self.get_parameter("turn_in_place_proximity").value)
        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.max_speed_scale = float(self.get_parameter("max_speed_scale").value)

        self.shark = SharkState()
        self.scan: LaserScan | None = None
        self.fish_state_payload = ""
        self.game_state_payload = ""
        self.last_target_log_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(String, "/phase2/fish_state_json", self._fish_callback, 10)
        self.create_subscription(String, "/phase2/game_state_json", self._game_callback, 10)
        self.create_timer(1.0 / control_hz, self._tick)

    def _odom_callback(self, msg: Odometry) -> None:
        self.shark.x = msg.pose.pose.position.x
        self.shark.y = msg.pose.pose.position.y
        self.shark.heading = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

    def _fish_callback(self, msg: String) -> None:
        self.fish_state_payload = msg.data

    def _game_callback(self, msg: String) -> None:
        self.game_state_payload = msg.data

    def _tick(self) -> None:
        if self.scan is None:
            return

        game_state = game_snapshot_from_json(self.game_state_payload)
        if not bool(game_state.get("sync_ready", True)):
            self.cmd_pub.publish(Twist())
            return
        cooldown = float(game_state.get("collision_cooldown", 0.0))
        if cooldown > 0.0:
            self.cmd_pub.publish(Twist())
            return

        fish_states = fish_states_from_json(self.fish_state_payload)
        target = self._select_target(fish_states)
        lidar = ObservationBuilder.preprocess_scan_ranges(
            self.scan.ranges,
            self.scan.range_min,
            self.scan.range_max,
            LIDAR_BIN_COUNT,
        )
        left_prox, center_prox, right_prox = ObservationBuilder.front_sector_proximities(lidar)

        cmd = Twist()
        if target is None:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
        else:
            bearing = angle_diff(bearing_xy(self.shark.x, self.shark.y, target["x"], target["y"]), self.shark.heading)
            target_turn = self.k_target * bearing
            avoid_turn = self.k_avoid * (right_prox - left_prox)
            cmd.angular.z = clamp(target_turn + avoid_turn, -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)

            if center_prox >= self.turn_in_place_proximity:
                cmd.linear.x = 0.0
            else:
                heading_factor = clamp(1.0 - 0.35 * abs(bearing), 0.25, 1.0)
                obstacle_factor = clamp(1.0 - center_prox, 0.2, 1.0)
                commanded_speed = self.cruise_speed * heading_factor + (SHARK_MAX_LINEAR_SPEED * self.max_speed_scale - self.cruise_speed) * obstacle_factor
                cmd.linear.x = clamp(commanded_speed, 0.0, SHARK_MAX_LINEAR_SPEED * self.max_speed_scale)

            now = self.get_clock().now()
            if (now - self.last_target_log_time).nanoseconds / 1e9 >= 1.0:
                distance = distance_xy(self.shark.x, self.shark.y, target["x"], target["y"])
                self.get_logger().info(
                    f"[AUTO] target={target['fish_id']} dist={distance:.2f} bearing={bearing:.2f} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
                )
                self.last_target_log_time = now

        self.cmd_pub.publish(cmd)

    def _select_target(self, fish_states: list[dict]) -> dict | None:
        best = None
        best_utility = -1.0
        score_map = {"tuna": 10.0, "sardine": 3.0, "seaweed": 1.0}

        for species in ("tuna", "sardine", "seaweed"):
            active = [fish for fish in fish_states if fish.get("active", False) and fish.get("species") == species]
            if not active:
                continue
            candidate = min(active, key=lambda fish: distance_xy(self.shark.x, self.shark.y, fish["x"], fish["y"]))
            utility = score_map[species] / (distance_xy(self.shark.x, self.shark.y, candidate["x"], candidate["y"]) + 1e-3)
            if utility > best_utility:
                best = candidate
                best_utility = utility
        return best


def main() -> None:
    rclpy.init()
    node = SharkAutoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
