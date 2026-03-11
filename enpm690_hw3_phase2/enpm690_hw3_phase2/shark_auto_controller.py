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
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("k_target", 1.8)
        self.declare_parameter("k_wall_avoid", 1.1)
        self.declare_parameter("behavior_caution", 1.0)
        self.declare_parameter("search_forward_speed", 0.28)
        self.declare_parameter("search_turn_speed", 0.85)
        self.declare_parameter("lock_linear_speed", 0.55)
        self.declare_parameter("lock_min_linear_speed", 0.12)
        self.declare_parameter("wall_stop_distance", 0.24)
        self.declare_parameter("wall_slow_distance", 0.70)
        self.declare_parameter("lidar_target_bias", 0.9)
        self.declare_parameter("lidar_cluster_jump_distance", 0.22)
        self.declare_parameter("lidar_cluster_min_beams", 2)
        self.declare_parameter("lidar_cluster_max_beams", 18)
        self.declare_parameter("lidar_cluster_max_width", 0.55)
        self.declare_parameter("lidar_cluster_max_range", 3.0)
        self.declare_parameter("lidar_front_fov_deg", 140.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.k_target = float(self.get_parameter("k_target").value)
        self.k_wall_avoid = float(self.get_parameter("k_wall_avoid").value)
        self.behavior_caution = float(self.get_parameter("behavior_caution").value)
        self.search_forward_speed = float(self.get_parameter("search_forward_speed").value)
        self.search_turn_speed = float(self.get_parameter("search_turn_speed").value)
        self.lock_linear_speed = float(self.get_parameter("lock_linear_speed").value)
        self.lock_min_linear_speed = float(self.get_parameter("lock_min_linear_speed").value)
        self.wall_stop_distance = float(self.get_parameter("wall_stop_distance").value)
        self.wall_slow_distance = float(self.get_parameter("wall_slow_distance").value)
        self.lidar_target_bias = float(self.get_parameter("lidar_target_bias").value)
        self.lidar_cluster_jump_distance = float(self.get_parameter("lidar_cluster_jump_distance").value)
        self.lidar_cluster_min_beams = int(self.get_parameter("lidar_cluster_min_beams").value)
        self.lidar_cluster_max_beams = int(self.get_parameter("lidar_cluster_max_beams").value)
        self.lidar_cluster_max_width = float(self.get_parameter("lidar_cluster_max_width").value)
        self.lidar_cluster_max_range = float(self.get_parameter("lidar_cluster_max_range").value)
        self.lidar_front_fov_deg = float(self.get_parameter("lidar_front_fov_deg").value)
        control_hz = float(self.get_parameter("control_hz").value)

        self.shark = SharkState()
        self.scan: LaserScan | None = None
        self.fish_state_payload = ""
        self.game_state_payload = ""
        self.mode = "searching"
        self.locked_target_id = ""
        self.last_log_time = self.get_clock().now()

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
        if float(game_state.get("collision_cooldown", 0.0)) > 0.0:
            self.cmd_pub.publish(Twist())
            return

        fish_states = fish_states_from_json(self.fish_state_payload)
        target = self._resolve_target(fish_states)
        sectors = self._sector_proximities(
            ObservationBuilder.preprocess_scan_ranges(
                self.scan.ranges,
                self.scan.range_min,
                self.scan.range_max,
                LIDAR_BIN_COUNT,
            )
        )
        wall_turn = self._wall_avoid_turn(sectors)
        front_distance = self._sector_distance(sectors.center)
        lidar_cluster = self._best_lidar_target_cluster()
        lidar_turn = 0.0 if lidar_cluster is None else self.lidar_target_bias * lidar_cluster.center_angle

        if target is None:
            cmd = self._build_search_command(wall_turn, front_distance)
        else:
            target_bearing = angle_diff(
                bearing_xy(self.shark.x, self.shark.y, target["x"], target["y"]),
                self.shark.heading,
            )
            cmd = self._build_lock_command(target_bearing, lidar_turn, wall_turn, front_distance)

        self._log_status(target, cmd)
        self.cmd_pub.publish(cmd)

    def _resolve_target(self, fish_states: list[dict]) -> dict | None:
        active_by_id = {fish["fish_id"]: fish for fish in fish_states if fish.get("active", False)}
        if self.locked_target_id in active_by_id:
            if self.mode != "lock_and_chase":
                self.mode = "lock_and_chase"
                self.get_logger().info(f"[AUTO] mode transition: searching -> lock_and_chase ({self.locked_target_id})")
            return active_by_id[self.locked_target_id]

        best = self._select_best_target(fish_states)
        if best is None:
            if self.mode != "searching":
                self.mode = "searching"
                self.get_logger().info("[AUTO] mode transition: lock_and_chase -> searching")
            self.locked_target_id = ""
            return None

        self.locked_target_id = str(best["fish_id"])
        if self.mode != "lock_and_chase":
            self.mode = "lock_and_chase"
            self.get_logger().info(f"[AUTO] mode transition: searching -> lock_and_chase ({self.locked_target_id})")
        return best

    def _select_best_target(self, fish_states: list[dict]) -> dict | None:
        best = None
        best_utility = float("-inf")
        score_map = {"tuna": 10.0, "sardine": 3.0, "seaweed": 1.0}
        for fish in fish_states:
            if not fish.get("active", False):
                continue
            distance = distance_xy(self.shark.x, self.shark.y, fish["x"], fish["y"])
            utility = score_map.get(fish["species"], 0.0) / (distance + 1e-3)
            if utility > best_utility:
                best = fish
                best_utility = utility
        return best

    def _build_search_command(self, wall_turn: float, front_distance: float) -> Twist:
        cmd = Twist()
        open_direction = 1.0 if wall_turn >= 0.0 else -1.0
        if front_distance <= self._effective_wall_stop_distance():
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(open_direction * max(self.search_turn_speed, 1.0), -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)
            return cmd

        speed_scale = clamp(front_distance / self._effective_wall_slow_distance(), 0.35, 1.0)
        cmd.linear.x = clamp(self.search_forward_speed * speed_scale, 0.0, SHARK_MAX_LINEAR_SPEED)
        cmd.angular.z = clamp(self.search_turn_speed + 0.8 * wall_turn, -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)
        return cmd

    def _build_lock_command(
        self,
        target_bearing: float,
        lidar_turn: float,
        wall_turn: float,
        front_distance: float,
    ) -> Twist:
        cmd = Twist()
        open_direction = 1.0 if wall_turn >= 0.0 else -1.0
        cmd.angular.z = clamp(
            self.k_target * target_bearing + 0.45 * lidar_turn + wall_turn,
            -SHARK_MAX_ANGULAR_SPEED,
            SHARK_MAX_ANGULAR_SPEED,
        )
        if front_distance <= self._effective_wall_stop_distance():
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(cmd.angular.z + 0.6 * open_direction, -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)
            return cmd

        heading_scale = clamp(1.0 - 0.45 * abs(target_bearing), 0.25, 1.0)
        clearance_scale = clamp(front_distance / self._effective_wall_slow_distance(), 0.35, 1.0)
        max_speed = clamp(self.lock_linear_speed * self._speed_scale(), 0.0, SHARK_MAX_LINEAR_SPEED)
        cmd.linear.x = clamp(max_speed * heading_scale * clearance_scale, self.lock_min_linear_speed, max_speed)
        return cmd

    def _wall_avoid_turn(self, sectors: SectorProximities) -> float:
        return self.k_wall_avoid * (
            (sectors.right - sectors.left) + 0.55 * (sectors.far_right - sectors.far_left)
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

    def _sector_distance(self, proximity: float) -> float:
        if self.scan is None:
            return float("inf")
        return max(self.scan.range_min, (1.0 - proximity) * self.scan.range_max)

    def _speed_scale(self) -> float:
        caution = clamp(self.behavior_caution, 0.6, 1.4)
        return clamp(1.15 - 0.18 * (caution - 1.0), 0.85, 1.20)

    def _effective_wall_stop_distance(self) -> float:
        caution = clamp(self.behavior_caution, 0.6, 1.4)
        return clamp(self.wall_stop_distance + 0.08 * (caution - 1.0), 0.18, 0.45)

    def _effective_wall_slow_distance(self) -> float:
        caution = clamp(self.behavior_caution, 0.6, 1.4)
        return clamp(self.wall_slow_distance + 0.15 * (caution - 1.0), 0.45, 1.20)

    def _best_lidar_target_cluster(self) -> LidarCluster | None:
        if self.scan is None:
            return None
        clusters = self._extract_lidar_clusters(self.scan)
        best_cluster = None
        best_score = float("-inf")
        for cluster in clusters:
            score = (
                2.2 / max(cluster.mean_range, 0.15)
                - 0.8 * abs(cluster.center_angle)
                - 0.6 * cluster.physical_width
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
            if self._cluster_break(scan.angle_increment, current_cluster[-1], point):
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
            mean_range = sum(ranges) / len(ranges)
            start_angle = cluster_points[0][2]
            end_angle = cluster_points[-1][2]
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

    def _log_status(self, target: dict | None, cmd: Twist) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 < 1.0:
            return
        if target is None:
            self.get_logger().info(
                f"[AUTO] mode={self.mode} target=none v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
            )
        else:
            distance = distance_xy(self.shark.x, self.shark.y, target["x"], target["y"])
            self.get_logger().info(
                f"[AUTO] mode={self.mode} target={target['fish_id']} dist={distance:.2f} "
                f"v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
            )
        self.last_log_time = now


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
