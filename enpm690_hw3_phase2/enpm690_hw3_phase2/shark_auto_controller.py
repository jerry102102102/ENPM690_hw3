#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .constants import LIDAR_BIN_COUNT, SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_LINEAR_SPEED
from .geometry_utils import clamp
from .observation_builder import ObservationBuilder
from .spawn_utils import game_snapshot_from_json


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
        self.declare_parameter("search_forward_speed", 0.22)
        self.declare_parameter("search_turn_speed", 0.70)
        self.declare_parameter("chase_linear_speed", 0.52)
        self.declare_parameter("chase_turn_gain", 1.8)
        self.declare_parameter("chase_min_linear_speed", 0.10)
        self.declare_parameter("chase_lost_timeout_sec", 0.6)
        self.declare_parameter("wall_turn_gain", 1.2)
        self.declare_parameter("wall_stop_distance", 0.26)
        self.declare_parameter("wall_slow_distance", 0.75)
        self.declare_parameter("lidar_cluster_jump_distance", 0.22)
        self.declare_parameter("lidar_cluster_min_beams", 2)
        self.declare_parameter("lidar_cluster_max_beams", 14)
        self.declare_parameter("lidar_cluster_max_width", 0.55)
        self.declare_parameter("lidar_cluster_max_range", 3.2)
        self.declare_parameter("lidar_front_fov_deg", 160.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.search_forward_speed = float(self.get_parameter("search_forward_speed").value)
        self.search_turn_speed = float(self.get_parameter("search_turn_speed").value)
        self.chase_linear_speed = float(self.get_parameter("chase_linear_speed").value)
        self.chase_turn_gain = float(self.get_parameter("chase_turn_gain").value)
        self.chase_min_linear_speed = float(self.get_parameter("chase_min_linear_speed").value)
        self.chase_lost_timeout_sec = float(self.get_parameter("chase_lost_timeout_sec").value)
        self.wall_turn_gain = float(self.get_parameter("wall_turn_gain").value)
        self.wall_stop_distance = float(self.get_parameter("wall_stop_distance").value)
        self.wall_slow_distance = float(self.get_parameter("wall_slow_distance").value)
        self.lidar_cluster_jump_distance = float(self.get_parameter("lidar_cluster_jump_distance").value)
        self.lidar_cluster_min_beams = int(self.get_parameter("lidar_cluster_min_beams").value)
        self.lidar_cluster_max_beams = int(self.get_parameter("lidar_cluster_max_beams").value)
        self.lidar_cluster_max_width = float(self.get_parameter("lidar_cluster_max_width").value)
        self.lidar_cluster_max_range = float(self.get_parameter("lidar_cluster_max_range").value)
        self.lidar_front_fov_deg = float(self.get_parameter("lidar_front_fov_deg").value)
        control_hz = float(self.get_parameter("control_hz").value)

        self.scan: LaserScan | None = None
        self.game_state_payload = ""
        self.mode = "searching"
        self.last_cluster_time = 0.0
        self.last_log_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(String, "/phase2/game_state_json", self._game_callback, 10)
        self.create_timer(1.0 / control_hz, self._tick)

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

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

        lidar = ObservationBuilder.preprocess_scan_ranges(
            self.scan.ranges,
            self.scan.range_min,
            self.scan.range_max,
            LIDAR_BIN_COUNT,
        )
        sectors = self._sector_proximities(lidar)
        front_distance = self._sector_distance(sectors.center)
        wall_turn = self._wall_avoid_turn(sectors)
        cluster = self._best_lidar_target_cluster()
        self._update_mode(cluster)

        if self.mode == "chasing" and cluster is not None:
            cmd = self._build_chase_command(cluster, wall_turn, front_distance)
        else:
            cmd = self._build_search_command(wall_turn, front_distance)

        self._log_status(cluster, cmd)
        self.cmd_pub.publish(cmd)

    def _update_mode(self, cluster: LidarCluster | None) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if cluster is not None:
            self.last_cluster_time = now
            if self.mode != "chasing":
                self.mode = "chasing"
                self.get_logger().info("[AUTO] mode transition: searching -> chasing")
            return

        if self.mode == "chasing" and now - self.last_cluster_time > self.chase_lost_timeout_sec:
            self.mode = "searching"
            self.get_logger().info("[AUTO] mode transition: chasing -> searching")

    def _build_search_command(self, wall_turn: float, front_distance: float) -> Twist:
        cmd = Twist()
        open_direction = 1.0 if wall_turn >= 0.0 else -1.0
        if front_distance <= self.wall_stop_distance:
            cmd.angular.z = clamp(open_direction * max(self.search_turn_speed, 0.9), -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)
            return cmd

        clearance_scale = clamp(front_distance / self.wall_slow_distance, 0.35, 1.0)
        cmd.linear.x = clamp(self.search_forward_speed * clearance_scale, 0.0, SHARK_MAX_LINEAR_SPEED)
        cmd.angular.z = clamp(self.search_turn_speed + wall_turn, -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)
        return cmd

    def _build_chase_command(self, cluster: LidarCluster, wall_turn: float, front_distance: float) -> Twist:
        cmd = Twist()
        open_direction = 1.0 if wall_turn >= 0.0 else -1.0
        cmd.angular.z = clamp(
            self.chase_turn_gain * cluster.center_angle + 0.8 * wall_turn,
            -SHARK_MAX_ANGULAR_SPEED,
            SHARK_MAX_ANGULAR_SPEED,
        )
        if front_distance <= self.wall_stop_distance:
            cmd.angular.z = clamp(cmd.angular.z + 0.7 * open_direction, -SHARK_MAX_ANGULAR_SPEED, SHARK_MAX_ANGULAR_SPEED)
            return cmd

        heading_scale = clamp(1.0 - 0.7 * abs(cluster.center_angle), 0.2, 1.0)
        clearance_scale = clamp(front_distance / self.wall_slow_distance, 0.30, 1.0)
        cmd.linear.x = clamp(
            self.chase_linear_speed * heading_scale * clearance_scale,
            self.chase_min_linear_speed,
            SHARK_MAX_LINEAR_SPEED,
        )
        return cmd

    def _wall_avoid_turn(self, sectors: SectorProximities) -> float:
        return self.wall_turn_gain * ((sectors.right - sectors.left) + 0.55 * (sectors.far_right - sectors.far_left))

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
        if not values:
            return 0.0
        return float(max(values))

    def _sector_distance(self, proximity: float) -> float:
        return max(self.scan.range_min, (1.0 - proximity) * self.scan.range_max)

    def _best_lidar_target_cluster(self) -> LidarCluster | None:
        if self.scan is None:
            return None
        clusters = self._extract_lidar_clusters(self.scan)
        best_cluster = None
        best_score = float("-inf")
        for cluster in clusters:
            score = 2.5 / max(cluster.mean_range, 0.15) - 0.9 * abs(cluster.center_angle) - 0.5 * cluster.physical_width
            if score > best_score:
                best_cluster = cluster
                best_score = score
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
            output.append(
                LidarCluster(
                    center_angle=sum(point[2] for point in cluster_points) / len(cluster_points),
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

    def _log_status(self, cluster: LidarCluster | None, cmd: Twist) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 < 1.0:
            return
        cluster_range = "none" if cluster is None else f"{cluster.mean_range:.2f}"
        cluster_angle = "none" if cluster is None else f"{cluster.center_angle:.2f}"
        self.get_logger().info(
            f"[AUTO] mode={self.mode} cluster_range={cluster_range} cluster_angle={cluster_angle} "
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
