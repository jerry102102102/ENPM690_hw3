#!/usr/bin/env python3

from __future__ import annotations

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

from .constants import PACMAN_GHOST_RADIUS, PACMAN_TIME_LIMIT, PacmanGameSnapshot, SHARK_CATCH_RADIUS
from .geometry_utils import distance_xy
from .ghost_manager import GhostManager
from .pellet_manager import PelletManager


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PacmanGameManager(Node):
    def __init__(self) -> None:
        super().__init__("pacman_game_manager")
        self.declare_parameter("game_dt", 0.1)
        self.declare_parameter("time_limit", PACMAN_TIME_LIMIT)
        self.declare_parameter("pellet_collect_radius", SHARK_CATCH_RADIUS)
        self.declare_parameter("ghost_collision_radius", PACMAN_GHOST_RADIUS + SHARK_CATCH_RADIUS)
        self.declare_parameter("score_topic", "/phase2/score")
        self.declare_parameter("time_topic", "/phase2/time_remaining")
        self.declare_parameter("game_state_topic", "/phase2/game_state_json")
        self.declare_parameter("pellet_state_topic", "/phase2/pellet_state_json")
        self.declare_parameter("ghost_state_topic", "/phase2/ghost_state_json")
        self.declare_parameter("mode", "teleop_play")

        self.dt = float(self.get_parameter("game_dt").value)
        self.time_limit = float(self.get_parameter("time_limit").value)
        self.pellet_collect_radius = float(self.get_parameter("pellet_collect_radius").value)
        self.ghost_collision_radius = float(self.get_parameter("ghost_collision_radius").value)
        self.mode = str(self.get_parameter("mode").value)

        self.pellet_manager = PelletManager()
        self.ghost_manager = GhostManager()

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.score = 0
        self.time_remaining = self.time_limit
        self.game_over = False
        self.victory = False

        self.score_pub = self.create_publisher(Int32, str(self.get_parameter("score_topic").value), 10)
        self.time_pub = self.create_publisher(Float32, str(self.get_parameter("time_topic").value), 10)
        self.game_state_pub = self.create_publisher(String, str(self.get_parameter("game_state_topic").value), 10)
        self.pellet_state_pub = self.create_publisher(String, str(self.get_parameter("pellet_state_topic").value), 10)
        self.ghost_state_pub = self.create_publisher(String, str(self.get_parameter("ghost_state_topic").value), 10)

        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_timer(self.dt, self._tick)

        self.reset_game()

    def reset_game(self) -> None:
        self.pellet_manager.reset()
        self.ghost_manager.reset()
        self.score = 0
        self.time_remaining = self.time_limit
        self.game_over = False
        self.victory = False
        self._publish_state()
        self.get_logger().info(f"[PACMAN] game started mode={self.mode}")

    def _odom_callback(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_heading = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def _tick(self) -> None:
        if self.game_over or self.victory:
            self._publish_state()
            return

        self.ghost_manager.update(self.dt)
        collected = self.pellet_manager.collect_near(self.robot_x, self.robot_y, self.pellet_collect_radius)
        if collected:
            self.score += sum(pellet.value for pellet in collected)
            self.get_logger().info(
                f"[PACMAN] collected={len(collected)} score={self.score} pellets_left={self.pellet_manager.active_count()}"
            )

        ghost = self.ghost_manager.ghost
        if distance_xy(self.robot_x, self.robot_y, ghost.x, ghost.y) <= self.ghost_collision_radius:
            self.game_over = True
            self.get_logger().info(f"[PACMAN] ghost collision game_over score={self.score}")

        self.time_remaining = max(0.0, self.time_remaining - self.dt)
        if self.pellet_manager.active_count() == 0:
            self.victory = True
            self.get_logger().info(f"[PACMAN] all pellets collected victory score={self.score}")
        elif self.time_remaining <= 0.0:
            self.game_over = True
            self.get_logger().info(f"[PACMAN] time up score={self.score}")

        self._publish_state()

    def _publish_state(self) -> None:
        self.score_pub.publish(Int32(data=self.score))
        self.time_pub.publish(Float32(data=float(self.time_remaining)))
        game_snapshot = PacmanGameSnapshot(
            mode=self.mode,
            score=self.score,
            time_remaining=self.time_remaining,
            pellets_remaining=self.pellet_manager.active_count(),
            game_over=self.game_over,
            victory=self.victory,
            sync_ready=True,
            collision_cooldown=0.0,
        )
        self.game_state_pub.publish(String(data=json.dumps(game_snapshot.to_dict(), separators=(",", ":"))))
        self.pellet_state_pub.publish(
            String(data=json.dumps([pellet.to_dict() for pellet in self.pellet_manager.pellets], separators=(",", ":")))
        )
        self.ghost_state_pub.publish(
            String(data=json.dumps(self.ghost_manager.ghost.to_dict(), separators=(",", ":")))
        )


def main() -> None:
    rclpy.init()
    node = PacmanGameManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
