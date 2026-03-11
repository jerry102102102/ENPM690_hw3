#!/usr/bin/env python3

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String

from .constants import (
    GAME_DT,
    GAME_DURATION_SECONDS,
    MODE_TRAIN,
    PHASE2_OBSTACLES,
    PHASE2_WORLD_BOUNDS,
    SHARK_RADIUS,
    SHARK_STUN_SECONDS,
    CatchEvent,
    GameSnapshot,
    SharkState,
)
from .fish_manager import FishManager
from .geometry_utils import bearing_xy, distance_xy, angle_diff
from .shark_collision_monitor import SharkCollisionMonitor
from .spawn_utils import fish_states_to_json, game_snapshot_to_json


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GameManager(Node):
    def __init__(self) -> None:
        super().__init__("game_manager")
        self.declare_parameter("mode", "teleop_play")
        self.declare_parameter("cmd_input_topic", "/cmd_vel_input")
        self.declare_parameter("cmd_output_topic", "/cmd_vel")
        self.declare_parameter("episode_duration", GAME_DURATION_SECONDS)
        self.declare_parameter("game_dt", GAME_DT)
        self.declare_parameter("score_topic", "/phase2/score")
        self.declare_parameter("time_topic", "/phase2/time_remaining")
        self.declare_parameter("mode_topic", "/phase2/mode")
        self.declare_parameter("catch_event_topic", "/phase2/catch_event")
        self.declare_parameter("fish_state_topic", "/phase2/fish_state_json")
        self.declare_parameter("game_state_topic", "/phase2/game_state_json")
        self.declare_parameter("fish_sync_ready_topic", "/phase2/fish_sync_ready")
        self.declare_parameter("auto_reset", True)

        self.mode = str(self.get_parameter("mode").value)
        self.cmd_input_topic = str(self.get_parameter("cmd_input_topic").value)
        self.cmd_output_topic = str(self.get_parameter("cmd_output_topic").value)
        self.episode_duration = float(self.get_parameter("episode_duration").value)
        self.dt = float(self.get_parameter("game_dt").value)
        self.auto_reset = bool(self.get_parameter("auto_reset").value)

        self.shark = SharkState()
        self.fish_manager = FishManager(PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES)
        self.collision_monitor = SharkCollisionMonitor(PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES, SHARK_RADIUS)
        self.latest_scan: LaserScan | None = None
        self.latest_cmd = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.score = 0
        self.time_remaining = self.episode_duration
        self.catch_counts = {"tuna": 0, "sardine": 0, "seaweed": 0}
        self.current_target_id = ""
        self.current_target_species = ""
        self.fish_sync_ready = self.mode == MODE_TRAIN
        self.waiting_for_sync = self.mode != MODE_TRAIN
        self._logged_waiting_for_sync = False

        self.score_pub = self.create_publisher(Int32, str(self.get_parameter("score_topic").value), 10)
        self.time_pub = self.create_publisher(Float32, str(self.get_parameter("time_topic").value), 10)
        self.mode_pub = self.create_publisher(String, str(self.get_parameter("mode_topic").value), 10)
        self.catch_event_pub = self.create_publisher(String, str(self.get_parameter("catch_event_topic").value), 10)
        self.fish_state_pub = self.create_publisher(String, str(self.get_parameter("fish_state_topic").value), 10)
        self.game_state_pub = self.create_publisher(String, str(self.get_parameter("game_state_topic").value), 10)
        self.cmd_output_pub = self.create_publisher(Twist, self.cmd_output_topic, 10)

        self.create_subscription(Twist, self.cmd_input_topic, self._cmd_input_callback, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("fish_sync_ready_topic").value), self._fish_sync_ready_callback, 10)
        self.create_timer(self.dt, self._tick)

        self.reset_episode(initial=True)

    def reset_episode(self, initial: bool = False) -> None:
        self.fish_manager.reset()
        self.score = 0
        self.time_remaining = self.episode_duration
        self.catch_counts = {"tuna": 0, "sardine": 0, "seaweed": 0}
        self.current_target_id = ""
        self.current_target_species = ""
        self.shark.collision_cooldown = 0.0
        self.waiting_for_sync = self.mode != MODE_TRAIN and not self.fish_sync_ready
        self._logged_waiting_for_sync = False
        if initial:
            self.get_logger().info(f"[GAME] episode started mode={self.mode}")
        else:
            self.get_logger().info(f"[GAME] episode reset mode={self.mode}")
        self._publish_status()

    def _cmd_input_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        self.shark.x = msg.pose.pose.position.x
        self.shark.y = msg.pose.pose.position.y
        self.shark.heading = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.shark.linear_speed = msg.twist.twist.linear.x
        self.shark.angular_speed = msg.twist.twist.angular.z

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _fish_sync_ready_callback(self, msg: Bool) -> None:
        if self.mode == MODE_TRAIN:
            return
        self.fish_sync_ready = bool(msg.data)
        if self.waiting_for_sync and msg.data:
            self.waiting_for_sync = False
            self._logged_waiting_for_sync = False
            self.get_logger().info("[GAME] fish visuals ready, starting episode")
        elif not msg.data:
            self.waiting_for_sync = True

    def _tick(self) -> None:
        if self.waiting_for_sync:
            self.cmd_output_pub.publish(Twist())
            self._update_target_snapshot()
            self._publish_status()
            if not self._logged_waiting_for_sync:
                self.get_logger().info("[GAME] waiting for fish visuals to finish spawning")
                self._logged_waiting_for_sync = True
            return

        collision = False
        if self.shark.collision_cooldown <= 0.0 and self.collision_monitor.check_collision(self.shark):
            self.shark.collision_cooldown = SHARK_STUN_SECONDS
            collision = True
            log = "[GAME] shark collision, stun=1.0s"
            self.get_logger().info(log)
            self.catch_event_pub.publish(String(data=log))

        if self.shark.collision_cooldown > 0.0:
            self.shark.collision_cooldown = max(0.0, self.shark.collision_cooldown - self.dt)
            gated_cmd = Twist()
        else:
            gated_cmd = self._fresh_cmd_or_zero()
        self.cmd_output_pub.publish(gated_cmd)

        self.fish_manager.update(self.dt, self.shark, immediate_respawn=self.mode == MODE_TRAIN)
        catches = self.fish_manager.detect_catches(self.shark, immediate_respawn=self.mode == MODE_TRAIN)
        if catches:
            self._handle_catches(catches)

        self.time_remaining = max(0.0, self.time_remaining - self.dt)
        self._update_target_snapshot()
        self._publish_status()

        if self.time_remaining <= 0.0:
            self.get_logger().info(f"[GAME] episode ended score={self.score} catches={json.dumps(self.catch_counts)}")
            if self.auto_reset:
                self.reset_episode()

    def _fresh_cmd_or_zero(self) -> Twist:
        age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds / 1e9
        if age > 0.5:
            return Twist()
        return self.latest_cmd

    def _handle_catches(self, catches: list[CatchEvent]) -> None:
        for event in catches:
            self.score += event.score_delta
            self.catch_counts[event.species] += 1
            text = f"[GAME] {event.species} caught, +{event.score_delta}, score={self.score}"
            self.get_logger().info(text)
            self.catch_event_pub.publish(String(data=text))

    def _update_target_snapshot(self) -> None:
        best_id = ""
        best_species = ""
        best_utility = -1.0
        for species, score in (("tuna", 10.0), ("sardine", 3.0), ("seaweed", 1.0)):
            fish = self.fish_manager.nearest_active_by_species(species, self.shark)
            if fish is None:
                continue
            utility = score / (distance_xy(self.shark.x, self.shark.y, fish.x, fish.y) + 1e-3)
            if utility > best_utility:
                best_utility = utility
                best_id = fish.fish_id
                best_species = fish.species
        self.current_target_id = best_id
        self.current_target_species = best_species

    def _publish_status(self) -> None:
        self.score_pub.publish(Int32(data=self.score))
        self.time_pub.publish(Float32(data=float(self.time_remaining)))
        self.mode_pub.publish(String(data=self.mode))
        self.fish_state_pub.publish(String(data=fish_states_to_json(self.fish_manager.fish)))
        snapshot = GameSnapshot(
            mode=self.mode,
            score=self.score,
            time_remaining=self.time_remaining,
            shark=self.shark,
            sync_ready=not self.waiting_for_sync,
            catch_counts=self.catch_counts,
            collision_cooldown=self.shark.collision_cooldown,
            current_target_id=self.current_target_id,
            current_target_species=self.current_target_species,
        )
        self.game_state_pub.publish(String(data=game_snapshot_to_json(snapshot)))


def main() -> None:
    rclpy.init()
    node = GameManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
