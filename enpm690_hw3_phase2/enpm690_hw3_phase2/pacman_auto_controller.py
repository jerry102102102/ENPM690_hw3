#!/usr/bin/env python3

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .constants import LIDAR_BIN_COUNT, SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_LINEAR_SPEED
from .geometry_utils import angle_diff, bearing_xy, clamp, distance_xy
from .observation_builder import ObservationBuilder


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PacmanAutoController(Node):
    def __init__(self) -> None:
        super().__init__("pacman_auto_controller")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("forward_speed", 0.28)
        self.declare_parameter("min_forward_speed", 0.12)
        self.declare_parameter("turn_gain", 1.7)
        self.declare_parameter("ghost_avoid_gain", 1.6)
        self.declare_parameter("ghost_avoid_distance", 1.2)
        self.declare_parameter("wall_avoid_gain", 1.2)
        self.declare_parameter("wall_stop_distance", 0.28)
        self.declare_parameter("wall_slow_distance", 0.85)
        self.declare_parameter("target_signal_gain", 0.9)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.min_forward_speed = float(self.get_parameter("min_forward_speed").value)
        self.turn_gain = float(self.get_parameter("turn_gain").value)
        self.ghost_avoid_gain = float(self.get_parameter("ghost_avoid_gain").value)
        self.ghost_avoid_distance = float(self.get_parameter("ghost_avoid_distance").value)
        self.wall_avoid_gain = float(self.get_parameter("wall_avoid_gain").value)
        self.wall_stop_distance = float(self.get_parameter("wall_stop_distance").value)
        self.wall_slow_distance = float(self.get_parameter("wall_slow_distance").value)
        self.target_signal_gain = float(self.get_parameter("target_signal_gain").value)
        control_hz = float(self.get_parameter("control_hz").value)
        self.get_logger().info(
            "autonomous params: "
            f"forward_speed={self.forward_speed:.2f}, "
            f"turn_gain={self.turn_gain:.2f}, "
            f"target_signal_gain={self.target_signal_gain:.2f}, "
            f"ghost_avoid_gain={self.ghost_avoid_gain:.2f}"
        )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.scan: LaserScan | None = None
        self.pellet_payload = "[]"
        self.ghost_payload = "{}"
        self.game_payload = "{}"
        self.last_log_time = self.get_clock().now()
        self.last_wait_log_time = self.get_clock().now()
        self.last_completion_log_time = self.get_clock().now()
        self.total_pellets = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, "/phase2/auto_status_json", 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(String, "/phase2/pellet_state_json", self._pellet_callback, 10)
        self.create_subscription(String, "/phase2/ghost_state_json", self._ghost_callback, 10)
        self.create_subscription(String, "/phase2/game_state_json", self._game_callback, 10)
        self.create_timer(1.0 / control_hz, self._tick)

    def _odom_callback(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_heading = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

    def _pellet_callback(self, msg: String) -> None:
        self.pellet_payload = msg.data

    def _ghost_callback(self, msg: String) -> None:
        self.ghost_payload = msg.data

    def _game_callback(self, msg: String) -> None:
        self.game_payload = msg.data

    def _tick(self) -> None:
        if self.scan is None:
            self._log_waiting("waiting for /scan")
            return

        game = json.loads(self.game_payload or "{}")
        if not game:
            self._log_waiting("waiting for /phase2/game_state_json")
            return
        if game.get("game_over", False) or game.get("victory", False):
            self.cmd_pub.publish(Twist())
            self._publish_status("", 0.0, game.get("score", 0), self.total_pellets, self.total_pellets, complete=True)
            self._log_waiting("game finished, publishing zero cmd")
            return

        all_pellets = json.loads(self.pellet_payload or "[]")
        if self.total_pellets <= 0:
            self.total_pellets = len(all_pellets)
        pellets = [pellet for pellet in all_pellets if pellet.get("active", False)]
        remaining = len(pellets)
        collected = max(0, self.total_pellets - remaining)
        if not pellets:
            self.cmd_pub.publish(Twist())
            self._publish_status("", 0.0, game.get("score", 0), collected, remaining, complete=True)
            self._log_completion()
            return

        target, target_bearing = self._select_target(pellets)

        ghost = json.loads(self.ghost_payload or "{}")
        ghost_turn = 0.0
        if ghost:
            ghost_distance = distance_xy(self.robot_x, self.robot_y, float(ghost.get("x", 0.0)), float(ghost.get("y", 0.0)))
            if ghost_distance < self.ghost_avoid_distance:
                ghost_bearing = angle_diff(
                    bearing_xy(self.robot_x, self.robot_y, float(ghost.get("x", 0.0)), float(ghost.get("y", 0.0))),
                    self.robot_heading,
                )
                ghost_turn = -self.ghost_avoid_gain * ghost_bearing

        lidar = ObservationBuilder.preprocess_scan_ranges(
            self.scan.ranges,
            self.scan.range_min,
            self.scan.range_max,
            LIDAR_BIN_COUNT,
        )
        left = self._slice_max(lidar, len(lidar) // 2 + 1, len(lidar) // 2 + 4)
        center = self._slice_max(lidar, len(lidar) // 2 - 1, len(lidar) // 2 + 2)
        right = self._slice_max(lidar, len(lidar) // 2 - 4, len(lidar) // 2 - 1)
        obstacle_turn, target_signal_heading, front_clearance = self._segment_and_select_heading(lidar, target_bearing)
        wall_turn = self.wall_avoid_gain * obstacle_turn
        front_distance = max(self.scan.range_min, (1.0 - center) * self.scan.range_max)
        obstacle_mix = clamp(1.0 - (front_clearance / max(self.wall_slow_distance, 1e-3)), 0.0, 1.0)
        signal_gain = self.target_signal_gain * obstacle_mix
        pursuit_heading = angle_diff(
            math.atan2(
                math.sin(target_bearing) + signal_gain * math.sin(target_signal_heading),
                math.cos(target_bearing) + signal_gain * math.cos(target_signal_heading),
            ),
            0.0,
        )

        cmd = Twist()
        cmd.angular.z = clamp(
            self.turn_gain * pursuit_heading + wall_turn + ghost_turn,
            -SHARK_MAX_ANGULAR_SPEED,
            SHARK_MAX_ANGULAR_SPEED,
        )
        if front_distance <= self.wall_stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(cmd.angular.z + (1.0 if wall_turn >= 0.0 else -1.0), -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)
        else:
            heading_scale = clamp(1.0 - 0.45 * abs(pursuit_heading), 0.20, 1.0)
            clearance_scale = clamp(min(front_distance, front_clearance) / self.wall_slow_distance, 0.20, 1.0)
            target_speed = self.forward_speed * heading_scale * clearance_scale
            cmd.linear.x = clamp(target_speed, self.min_forward_speed, SHARK_MAX_LINEAR_SPEED)

        self._publish_status(target["pellet_id"], target_bearing, game.get("score", 0), collected, remaining, complete=False)
        self._log_status(target, pursuit_heading, front_distance, left, center, right, wall_turn, ghost_turn, cmd)
        self.cmd_pub.publish(cmd)

    def _slice_max(self, lidar: list[float], start: int, end: int) -> float:
        values = lidar[max(0, start) : min(len(lidar), end)]
        if len(values) == 0:
            return 0.0
        return float(max(values))

    def _segment_and_select_heading(self, lidar: list[float], target_bearing: float) -> tuple[float, float, float]:
        if not lidar:
            return 0.0, 0.0, self.wall_slow_distance

        n = len(lidar)
        front_clearance = self.scan.range_max
        left_risk = 0.0
        right_risk = 0.0
        left_count = 0
        right_count = 0
        best_score = -1.0
        best_angle = target_bearing
        best_alignment = -1.0
        best_distance = float("inf")
        best_idx = 0

        for idx, proximity in enumerate(lidar):
            rel = -math.pi + (2.0 * math.pi * idx / max(1, n - 1))
            risk = clamp(float(proximity), 0.0, 1.0)
            clearance = 1.0 - risk
            alignment = max(0.0, math.cos(angle_diff(rel, target_bearing)))
            forward_weight = max(0.0, math.cos(rel))
            score = clearance * (0.70 * alignment + 0.30 * forward_weight)

            if abs(rel) < 0.35:
                bin_distance = max(self.scan.range_min, (1.0 - risk) * self.scan.range_max)
                front_clearance = min(front_clearance, bin_distance)
            if 0.25 <= rel <= 1.20:
                left_risk += risk
                left_count += 1
            if -1.20 <= rel <= -0.25:
                right_risk += risk
                right_count += 1

            diff = abs(angle_diff(rel, target_bearing))
            if score > best_score or (
                abs(score - best_score) <= 1e-6
                and (alignment > best_alignment or (abs(alignment - best_alignment) <= 1e-6 and (diff < best_distance or (abs(diff - best_distance) <= 1e-6 and idx < best_idx))))
            ):
                best_score = score
                best_alignment = alignment
                best_distance = diff
                best_idx = idx
                best_angle = rel

        left = left_risk / max(1, left_count)
        right = right_risk / max(1, right_count)
        obstacle_turn = right - left
        return obstacle_turn, best_angle, front_clearance

    def _select_target(self, pellets: list[dict]) -> tuple[dict, float]:
        best_target = pellets[0]
        best_bearing = 0.0
        best_cost = float("inf")
        for pellet in pellets:
            bearing = angle_diff(
                bearing_xy(self.robot_x, self.robot_y, float(pellet["x"]), float(pellet["y"])),
                self.robot_heading,
            )
            distance = distance_xy(self.robot_x, self.robot_y, float(pellet["x"]), float(pellet["y"]))
            cost = distance + 0.8 * abs(bearing)
            if cost < best_cost:
                best_target = pellet
                best_bearing = bearing
                best_cost = cost
        return best_target, best_bearing

    def _publish_status(
        self,
        target_id: str,
        target_bearing: float,
        score: int,
        collected: int,
        remaining: int,
        complete: bool,
    ) -> None:
        payload = {
            "target_id": target_id,
            "target_bearing": float(target_bearing),
            "score": int(score),
            "collected": int(collected),
            "remaining": int(remaining),
            "complete": bool(complete),
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def _log_completion(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_completion_log_time).nanoseconds / 1e9 < 1.0:
            return
        self.get_logger().info("[AUTO] completion: all pellets collected")
        self.last_completion_log_time = now

    def _log_status(
        self,
        target: dict,
        target_bearing: float,
        front_distance: float,
        left: float,
        center: float,
        right: float,
        wall_turn: float,
        ghost_turn: float,
        cmd: Twist,
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 < 1.0:
            return
        distance = distance_xy(self.robot_x, self.robot_y, float(target["x"]), float(target["y"]))
        self.get_logger().info(
            f"[AUTO] target={target['pellet_id']} dist={distance:.2f} bearing={target_bearing:.2f} "
            f"front={front_distance:.2f} lidar(l={left:.2f},c={center:.2f},r={right:.2f}) "
            f"turns(wall={wall_turn:.2f},ghost={ghost_turn:.2f}) cmd(v={cmd.linear.x:.2f},w={cmd.angular.z:.2f})"
        )
        self.last_log_time = now

    def _log_waiting(self, message: str) -> None:
        now = self.get_clock().now()
        if (now - self.last_wait_log_time).nanoseconds / 1e9 < 1.0:
            return
        self.get_logger().info(f"[AUTO] {message}")
        self.last_wait_log_time = now


def main() -> None:
    rclpy.init()
    node = PacmanAutoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
