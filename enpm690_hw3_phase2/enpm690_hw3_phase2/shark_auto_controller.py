#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass

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


@dataclass(frozen=True)
class SectorProximities:
    far_left: float
    left: float
    center: float
    right: float
    far_right: float

    @property
    def obstacle_strength(self) -> float:
        return max(self.far_left, self.left, self.center, self.right, self.far_right)


@dataclass(frozen=True)
class LidarCluster:
    center_angle: float
    mean_range: float
    beam_count: int
    physical_width: float


class SharkAutoController(Node):
    def __init__(self) -> None:
        super().__init__("shark_auto_controller")
        self.declare_parameter("cmd_topic", "/cmd_vel_input")
        self.declare_parameter("k_target", 1.6)
        self.declare_parameter("k_avoid", 1.4)
        self.declare_parameter("k_avoid_far", 0.9)
        self.declare_parameter("k_center_push", 1.8)
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("behavior_caution", 1.0)
        self.declare_parameter("turn_in_place_distance", 0.32)
        self.declare_parameter("cruise_speed", 0.22)
        self.declare_parameter("max_speed_scale", 1.0)
        self.declare_parameter("target_lock_seconds", 0.9)
        self.declare_parameter("avoid_enter_distance", 0.80)
        self.declare_parameter("avoid_exit_distance", 0.95)
        self.declare_parameter("side_clear_distance", 0.72)
        self.declare_parameter("avoid_commit_seconds", 0.9)
        self.declare_parameter("recover_seconds", 0.5)
        self.declare_parameter("avoid_turn_speed", 1.25)
        self.declare_parameter("avoid_forward_speed", 0.16)
        self.declare_parameter("recover_turn_speed", 0.55)
        self.declare_parameter("lidar_target_bias", 0.9)
        self.declare_parameter("lidar_cluster_jump_distance", 0.22)
        self.declare_parameter("lidar_cluster_min_beams", 2)
        self.declare_parameter("lidar_cluster_max_beams", 18)
        self.declare_parameter("lidar_cluster_max_width", 0.55)
        self.declare_parameter("lidar_cluster_max_range", 3.0)
        self.declare_parameter("lidar_front_fov_deg", 140.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.k_target = float(self.get_parameter("k_target").value)
        self.k_avoid = float(self.get_parameter("k_avoid").value)
        self.k_avoid_far = float(self.get_parameter("k_avoid_far").value)
        self.k_center_push = float(self.get_parameter("k_center_push").value)
        control_hz = float(self.get_parameter("control_hz").value)
        self.behavior_caution = float(self.get_parameter("behavior_caution").value)
        self.turn_in_place_distance = float(self.get_parameter("turn_in_place_distance").value)
        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.max_speed_scale = float(self.get_parameter("max_speed_scale").value)
        self.target_lock_seconds = float(self.get_parameter("target_lock_seconds").value)
        self.avoid_enter_distance = float(self.get_parameter("avoid_enter_distance").value)
        self.avoid_exit_distance = float(self.get_parameter("avoid_exit_distance").value)
        self.side_clear_distance = float(self.get_parameter("side_clear_distance").value)
        self.avoid_commit_seconds = float(self.get_parameter("avoid_commit_seconds").value)
        self.recover_seconds = float(self.get_parameter("recover_seconds").value)
        self.avoid_turn_speed = float(self.get_parameter("avoid_turn_speed").value)
        self.avoid_forward_speed = float(self.get_parameter("avoid_forward_speed").value)
        self.recover_turn_speed = float(self.get_parameter("recover_turn_speed").value)
        self.lidar_target_bias = float(self.get_parameter("lidar_target_bias").value)
        self.lidar_cluster_jump_distance = float(self.get_parameter("lidar_cluster_jump_distance").value)
        self.lidar_cluster_min_beams = int(self.get_parameter("lidar_cluster_min_beams").value)
        self.lidar_cluster_max_beams = int(self.get_parameter("lidar_cluster_max_beams").value)
        self.lidar_cluster_max_width = float(self.get_parameter("lidar_cluster_max_width").value)
        self.lidar_cluster_max_range = float(self.get_parameter("lidar_cluster_max_range").value)
        self.lidar_front_fov_deg = float(self.get_parameter("lidar_front_fov_deg").value)

        self.shark = SharkState()
        self.scan: LaserScan | None = None
        self.fish_state_payload = ""
        self.game_state_payload = ""
        self.last_target_log_time = self.get_clock().now()
        self.mode = "chase"
        self.avoid_direction = 1.0
        self.avoid_until = 0.0
        self.recover_until = 0.0
        self.locked_target_id = ""
        self.target_lock_until = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(String, "/phase2/fish_state_json", self._fish_callback, 10)
        self.create_subscription(String, "/phase2/game_state_json", self._game_callback, 10)
        self.create_timer(1.0 / control_hz, self._tick)
        self.get_logger().info(
            f"[AUTO] behavior_caution={self.behavior_caution:.2f} "
            "(higher means earlier avoidance and lower forward speed)"
        )

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
        sectors = self._sector_proximities(lidar)
        lidar_cluster = self._best_lidar_target_cluster()
        target_bearing = 0.0 if target is None else angle_diff(
            bearing_xy(self.shark.x, self.shark.y, target["x"], target["y"]),
            self.shark.heading,
        )
        target_turn = self.k_target * target_bearing
        lidar_turn = 0.0 if lidar_cluster is None else self.lidar_target_bias * lidar_cluster.center_angle
        self._update_mode(sectors)
        reactive_turn = self._braitenberg_turn(sectors)

        cmd = Twist()
        if target is None:
            cmd.linear.x = self._effective_cruise_speed()
            cmd.angular.z = clamp(lidar_turn + 0.7 * reactive_turn, -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)
        elif self.mode == "avoid":
            cmd = self._build_avoid_command(reactive_turn, sectors)
        elif self.mode == "recover":
            cmd = self._build_recover_command(target_turn, lidar_turn, reactive_turn, sectors)
        else:
            cmd = self._build_chase_command(target_turn, lidar_turn, reactive_turn, target_bearing, sectors)

        now = self.get_clock().now()
        if target is not None and (now - self.last_target_log_time).nanoseconds / 1e9 >= 1.0:
            distance = distance_xy(self.shark.x, self.shark.y, target["x"], target["y"])
            self.get_logger().info(
                f"[AUTO] mode={self.mode} target={target['fish_id']} dist={distance:.2f} "
                f"bearing={target_bearing:.2f} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
            )
            self.last_target_log_time = now

        self.cmd_pub.publish(cmd)

    def _select_target(self, fish_states: list[dict]) -> dict | None:
        now = self._now_seconds()
        active_by_id = {
            fish["fish_id"]: fish
            for fish in fish_states
            if fish.get("active", False)
        }

        if self.locked_target_id in active_by_id and now < self.target_lock_until:
            return active_by_id[self.locked_target_id]

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
        if best is None:
            self.locked_target_id = ""
            self.target_lock_until = 0.0
            return None

        self.locked_target_id = str(best["fish_id"])
        self.target_lock_until = now + self.target_lock_seconds
        return best

    def _build_chase_command(
        self,
        target_turn: float,
        lidar_turn: float,
        reactive_turn: float,
        target_bearing: float,
        sectors: SectorProximities,
    ) -> Twist:
        cmd = Twist()
        cmd.angular.z = clamp(
            target_turn + 0.55 * lidar_turn + 0.65 * reactive_turn,
            -SHARK_MAX_ANGULAR_SPEED,
            SHARK_MAX_ANGULAR_SPEED,
        )
        if self._sector_distance(sectors.center) <= self._effective_turn_in_place_distance():
            cmd.linear.x = 0.0
            return cmd

        heading_factor = clamp(1.0 - 0.42 * abs(target_bearing), 0.20, 1.0)
        clearance_factor = clamp(1.0 - 0.85 * sectors.obstacle_strength, 0.15, 1.0)
        max_speed = self._effective_max_speed()
        cmd.linear.x = clamp(max_speed * heading_factor * clearance_factor, 0.0, max_speed)
        return cmd

    def _build_avoid_command(self, reactive_turn: float, sectors: SectorProximities) -> Twist:
        cmd = Twist()
        cmd.angular.z = clamp(
            self.avoid_turn_speed * self.avoid_direction + 0.7 * reactive_turn,
            -SHARK_MAX_ANGULAR_SPEED,
            SHARK_MAX_ANGULAR_SPEED,
        )
        center_distance = self._sector_distance(sectors.center)
        if center_distance <= max(0.18, 0.7 * self._effective_turn_in_place_distance()):
            cmd.linear.x = 0.0
            return cmd
        cmd.linear.x = clamp(
            self._effective_avoid_forward_speed() * clamp(center_distance / self._effective_avoid_enter_distance(), 0.35, 1.0),
            0.04,
            self._effective_avoid_forward_speed(),
        )
        return cmd

    def _build_recover_command(
        self,
        target_turn: float,
        lidar_turn: float,
        reactive_turn: float,
        sectors: SectorProximities,
    ) -> Twist:
        cmd = Twist()
        cmd.angular.z = clamp(
            0.45 * target_turn + 0.45 * lidar_turn + 0.5 * reactive_turn + self.recover_turn_speed * self.avoid_direction,
            -SHARK_MAX_ANGULAR_SPEED,
            SHARK_MAX_ANGULAR_SPEED,
        )
        max_speed = self._effective_max_speed()
        cmd.linear.x = clamp(
            0.6 * self._effective_cruise_speed() * (1.0 - 0.5 * sectors.obstacle_strength),
            0.08,
            max_speed,
        )
        return cmd

    def _update_mode(self, sectors: SectorProximities) -> None:
        now = self._now_seconds()
        center_distance = self._sector_distance(sectors.center)
        obstacle_distance = self._sector_distance(sectors.obstacle_strength)
        side_distance = min(self._sector_distance(sectors.left), self._sector_distance(sectors.right))
        if self.mode == "avoid":
            if now >= self.avoid_until and (
                center_distance >= self._effective_avoid_exit_distance()
                and side_distance >= self._effective_side_clear_distance()
            ):
                self.mode = "recover"
                self.recover_until = now + self.recover_seconds
                self.get_logger().info("[AUTO] mode transition: avoid -> recover")
            return

        if self.mode == "recover":
            if obstacle_distance <= self._effective_avoid_enter_distance():
                self._enter_avoid_mode(sectors, now)
            elif now >= self.recover_until:
                self.mode = "chase"
                self.get_logger().info("[AUTO] mode transition: recover -> chase")
            return

        if obstacle_distance <= self._effective_avoid_enter_distance():
            self._enter_avoid_mode(sectors, now)

    def _enter_avoid_mode(self, sectors: SectorProximities, now: float) -> None:
        self.mode = "avoid"
        right_pressure = sectors.right + 0.5 * sectors.far_right
        left_pressure = sectors.left + 0.5 * sectors.far_left
        self.avoid_direction = 1.0 if right_pressure >= left_pressure else -1.0
        self.avoid_until = now + self.avoid_commit_seconds
        direction = "left" if self.avoid_direction > 0.0 else "right"
        self.get_logger().info(f"[AUTO] mode transition: chase/recover -> avoid ({direction})")

    def _braitenberg_turn(self, sectors: SectorProximities) -> float:
        center_push = self.k_center_push * self.avoid_direction * sectors.center if self.mode != "chase" else 0.0
        return (
            self.k_avoid * (sectors.right - sectors.left)
            + self.k_avoid_far * (sectors.far_right - sectors.far_left)
            + center_push
        )

    def _sector_proximities(self, lidar: list[float]) -> SectorProximities:
        front_center = len(lidar) // 2
        return SectorProximities(
            far_left=self._slice_max(lidar, front_center + 4, front_center + 8),
            left=self._slice_max(lidar, front_center + 1, front_center + 4),
            center=self._slice_max(lidar, front_center - 1, front_center + 2),
            right=self._slice_max(lidar, front_center - 4, front_center - 1),
            far_right=self._slice_max(lidar, front_center - 8, front_center - 4),
        )

    def _slice_max(self, lidar: list[float], start: int, end: int) -> float:
        values = lidar[max(0, start) : min(len(lidar), end)]
        if len(values) == 0:
            return 0.0
        return float(max(values))

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _effective_behavior_caution(self) -> float:
        return clamp(self.behavior_caution, 0.6, 1.6)

    def _effective_max_speed(self) -> float:
        caution = self._effective_behavior_caution()
        speed_scale = self.max_speed_scale * (1.10 - 0.30 * (caution - 1.0))
        return SHARK_MAX_LINEAR_SPEED * clamp(speed_scale, 0.55, 1.10)

    def _effective_cruise_speed(self) -> float:
        caution = self._effective_behavior_caution()
        scale = 1.08 - 0.28 * (caution - 1.0)
        return clamp(self.cruise_speed * scale, 0.10, SHARK_MAX_LINEAR_SPEED)

    def _effective_avoid_forward_speed(self) -> float:
        caution = self._effective_behavior_caution()
        scale = 1.05 - 0.35 * (caution - 1.0)
        return clamp(self.avoid_forward_speed * scale, 0.04, 0.20)

    def _effective_turn_in_place_distance(self) -> float:
        caution = self._effective_behavior_caution()
        return clamp(self.turn_in_place_distance + 0.12 * (caution - 1.0), 0.22, 0.55)

    def _effective_avoid_enter_distance(self) -> float:
        caution = self._effective_behavior_caution()
        return clamp(self.avoid_enter_distance + 0.25 * (caution - 1.0), 0.55, 1.20)

    def _effective_avoid_exit_distance(self) -> float:
        caution = self._effective_behavior_caution()
        return clamp(self.avoid_exit_distance + 0.20 * (caution - 1.0), 0.70, 1.30)

    def _effective_side_clear_distance(self) -> float:
        caution = self._effective_behavior_caution()
        return clamp(self.side_clear_distance + 0.16 * (caution - 1.0), 0.50, 1.10)

    def _sector_distance(self, proximity: float) -> float:
        if self.scan is None:
            return float("inf")
        return max(self.scan.range_min, (1.0 - proximity) * self.scan.range_max)

    def _best_lidar_target_cluster(self) -> LidarCluster | None:
        if self.scan is None:
            return None
        clusters = self._extract_lidar_clusters(self.scan)
        best_cluster = None
        best_score = float("-inf")
        for cluster in clusters:
            score = (
                2.2 / max(cluster.mean_range, 0.15)
                - 0.9 * abs(cluster.center_angle)
                - 0.7 * cluster.physical_width
            )
            if score > best_score:
                best_score = score
                best_cluster = cluster
        return best_cluster

    def _extract_lidar_clusters(self, scan: LaserScan) -> list[LidarCluster]:
        front_half_fov = math.radians(self.lidar_front_fov_deg) / 2.0
        points: list[tuple[int, float, float]] = []
        for index, raw_range in enumerate(scan.ranges):
            if not math.isfinite(raw_range):
                continue
            if raw_range < scan.range_min or raw_range > min(scan.range_max, self.lidar_cluster_max_range):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > front_half_fov:
                continue
            points.append((index, raw_range, angle))
        if not points:
            return []

        clusters: list[list[tuple[int, float, float]]] = []
        current_cluster = [points[0]]
        for point in points[1:]:
            prev_point = current_cluster[-1]
            if self._cluster_break(scan.angle_increment, prev_point, point):
                clusters.append(current_cluster)
                current_cluster = [point]
            else:
                current_cluster.append(point)
        clusters.append(current_cluster)

        output: list[LidarCluster] = []
        for cluster_points in clusters:
            if not (self.lidar_cluster_min_beams <= len(cluster_points) <= self.lidar_cluster_max_beams):
                continue
            ranges = [point[1] for point in cluster_points]
            start_angle = cluster_points[0][2]
            end_angle = cluster_points[-1][2]
            mean_range = sum(ranges) / len(ranges)
            angular_span = abs(end_angle - start_angle)
            physical_width = 2.0 * mean_range * math.sin(angular_span / 2.0)
            if physical_width > self.lidar_cluster_max_width:
                continue
            center_angle = sum(point[2] for point in cluster_points) / len(cluster_points)
            output.append(
                LidarCluster(
                    center_angle=center_angle,
                    mean_range=mean_range,
                    beam_count=len(cluster_points),
                    physical_width=physical_width,
                )
            )
        return output

    def _cluster_break(
        self,
        angle_increment: float,
        previous_point: tuple[int, float, float],
        current_point: tuple[int, float, float],
    ) -> bool:
        prev_index, prev_range, _ = previous_point
        curr_index, curr_range, _ = current_point
        beam_gap = curr_index - prev_index
        if beam_gap > 1:
            return True
        angle_gap = max(angle_increment * beam_gap, 1e-6)
        euclidean_gap = math.sqrt(
            prev_range * prev_range
            + curr_range * curr_range
            - 2.0 * prev_range * curr_range * math.cos(angle_gap)
        )
        return euclidean_gap > self.lidar_cluster_jump_distance


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
