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
    CatchEvent,
)
from .game_core import GameCore
from .fish_manager import FishManager
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
        self.declare_parameter("respawn_caught_fish", True)
        self.declare_parameter("tuna_speed_scale", 0.8)
        self.declare_parameter("sardine_speed_scale", 0.4)
        self.declare_parameter("seaweed_speed_scale", 0.0)

        self.mode = str(self.get_parameter("mode").value)
        self.cmd_input_topic = str(self.get_parameter("cmd_input_topic").value)
        self.cmd_output_topic = str(self.get_parameter("cmd_output_topic").value)
        self.episode_duration = float(self.get_parameter("episode_duration").value)
        self.dt = float(self.get_parameter("game_dt").value)
        self.auto_reset = bool(self.get_parameter("auto_reset").value)
        self.respawn_caught_fish = bool(self.get_parameter("respawn_caught_fish").value)
        species_speed_scales = {
            "tuna": float(self.get_parameter("tuna_speed_scale").value),
            "sardine": float(self.get_parameter("sardine_speed_scale").value),
            "seaweed": float(self.get_parameter("seaweed_speed_scale").value),
        }

        fish_manager = FishManager(self._default_bounds(), self._default_obstacles(), species_speed_scales=species_speed_scales)
        self.game = GameCore(episode_duration=self.episode_duration, dt=self.dt, fish_manager=fish_manager)
        self.latest_scan: LaserScan | None = None
        self.latest_cmd = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.fish_sync_ready = self.mode == MODE_TRAIN
        self.waiting_for_sync = self.mode != MODE_TRAIN
        self._logged_waiting_for_sync = False
        self.completed = False
        self.start_time = self.get_clock().now()

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
        self.game.reset()
        self.completed = False
        self.start_time = self.get_clock().now()
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
        self.game.shark.x = msg.pose.pose.position.x
        self.game.shark.y = msg.pose.pose.position.y
        self.game.shark.heading = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.game.shark.linear_speed = msg.twist.twist.linear.x
        self.game.shark.angular_speed = msg.twist.twist.angular.z

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
        if self.completed:
            self.cmd_output_pub.publish(Twist())
            self._publish_status()
            return

        if self.waiting_for_sync:
            self.cmd_output_pub.publish(Twist())
            self._publish_status()
            if not self._logged_waiting_for_sync:
                self.get_logger().info("[GAME] waiting for fish visuals to finish spawning")
                self._logged_waiting_for_sync = True
            return

        collision = self.game.trigger_collision_if_needed()
        if collision:
            log = "[GAME] shark collision, stun=1.0s"
            self.get_logger().info(log)
            self.catch_event_pub.publish(String(data=log))

        if self.game.consume_cooldown_tick():
            gated_cmd = Twist()
        else:
            gated_cmd = self._fresh_cmd_or_zero()
        self.cmd_output_pub.publish(gated_cmd)

        catches = self.game.advance_episode_with_options(
            immediate_respawn=self.mode == MODE_TRAIN,
            respawn_enabled=self.respawn_caught_fish or self.mode == MODE_TRAIN,
        )
        if catches:
            self._handle_catches(catches)

        self._publish_status()

        if self.game.fish_manager.active_count() == 0:
            self._complete_run()
            return

        if self.game.time_remaining <= 0.0:
            self.get_logger().info(f"[GAME] episode ended score={self.game.score} catches={json.dumps(self.game.catch_counts)}")
            if self.auto_reset:
                self.reset_episode()

    def _fresh_cmd_or_zero(self) -> Twist:
        age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds / 1e9
        if age > 0.5:
            return Twist()
        return self.latest_cmd

    def _handle_catches(self, catches: list[CatchEvent]) -> None:
        for event in catches:
            text = f"[GAME] {event.species} caught, +{event.score_delta}, score={self.game.score}"
            self.get_logger().info(text)
            self.catch_event_pub.publish(String(data=text))

    def _publish_status(self) -> None:
        self.score_pub.publish(Int32(data=self.game.score))
        self.time_pub.publish(Float32(data=float(self.game.time_remaining)))
        self.mode_pub.publish(String(data=self.mode))
        self.fish_state_pub.publish(String(data=fish_states_to_json(self.game.fish_manager.fish)))
        snapshot = self.game.build_snapshot(mode=self.mode, sync_ready=not self.waiting_for_sync)
        self.game_state_pub.publish(String(data=game_snapshot_to_json(snapshot)))

    def _complete_run(self) -> None:
        self.completed = True
        self.cmd_output_pub.publish(Twist())
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        summary = (
            "[GAME] run complete "
            f"elapsed={elapsed:.1f}s score={self.game.score} "
            f"tuna={self.game.catch_counts['tuna']} sardine={self.game.catch_counts['sardine']} "
            f"seaweed={self.game.catch_counts['seaweed']}"
        )
        self.get_logger().info(summary)
        self.catch_event_pub.publish(String(data=summary))

    def _default_bounds(self):
        from .constants import PHASE2_WORLD_BOUNDS

        return PHASE2_WORLD_BOUNDS

    def _default_obstacles(self):
        from .constants import PHASE2_OBSTACLES

        return PHASE2_OBSTACLES


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
