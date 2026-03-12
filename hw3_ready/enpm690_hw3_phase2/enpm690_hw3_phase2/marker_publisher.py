#!/usr/bin/env python3

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .constants import PACMAN_GHOST_RADIUS, PACMAN_PELLET_RADIUS, PACMAN_WORLD_BOUNDS


class MarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("marker_publisher")
        self.declare_parameter("marker_topic", "/phase2/markers")
        self.marker_topic = str(self.get_parameter("marker_topic").value)

        self.pellet_payload = "[]"
        self.ghost_payload = "{}"
        self.game_payload = "{}"
        self.auto_payload = "{}"

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(String, "/phase2/pellet_state_json", self._pellet_callback, 10)
        self.create_subscription(String, "/phase2/ghost_state_json", self._ghost_callback, 10)
        self.create_subscription(String, "/phase2/game_state_json", self._game_callback, 10)
        self.create_subscription(String, "/phase2/auto_status_json", self._auto_callback, 10)
        self.create_timer(0.1, self._tick)

    def _pellet_callback(self, msg: String) -> None:
        self.pellet_payload = msg.data

    def _ghost_callback(self, msg: String) -> None:
        self.ghost_payload = msg.data

    def _game_callback(self, msg: String) -> None:
        self.game_payload = msg.data

    def _auto_callback(self, msg: String) -> None:
        self.auto_payload = msg.data

    def _tick(self) -> None:
        pellets = json.loads(self.pellet_payload or "[]")
        ghost = json.loads(self.ghost_payload or "{}")
        game = json.loads(self.game_payload or "{}")
        auto = json.loads(self.auto_payload or "{}")
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for index, pellet in enumerate(pellets):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = stamp
            marker.ns = "pellets"
            marker.id = index
            marker.action = Marker.ADD if pellet.get("active", False) else Marker.DELETE
            marker.type = Marker.SPHERE
            marker.pose.position.x = float(pellet.get("x", 0.0))
            marker.pose.position.y = float(pellet.get("y", 0.0))
            marker.pose.position.z = 0.10
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * PACMAN_PELLET_RADIUS
            marker.scale.y = 2.0 * PACMAN_PELLET_RADIUS
            marker.scale.z = 2.0 * PACMAN_PELLET_RADIUS
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.15
            marker.color.a = 1.0
            markers.markers.append(marker)

        ghost_marker = Marker()
        ghost_marker.header.frame_id = "odom"
        ghost_marker.header.stamp = stamp
        ghost_marker.ns = "ghost"
        ghost_marker.id = 100
        ghost_marker.action = Marker.ADD
        ghost_marker.type = Marker.CYLINDER
        ghost_marker.pose.position.x = float(ghost.get("x", 0.0))
        ghost_marker.pose.position.y = float(ghost.get("y", 0.0))
        ghost_marker.pose.position.z = 0.22
        ghost_marker.pose.orientation.w = 1.0
        ghost_marker.scale.x = 2.0 * PACMAN_GHOST_RADIUS
        ghost_marker.scale.y = 2.0 * PACMAN_GHOST_RADIUS
        ghost_marker.scale.z = 0.44
        ghost_marker.color.r = 0.95
        ghost_marker.color.g = 0.20
        ghost_marker.color.b = 0.20
        ghost_marker.color.a = 0.95
        markers.markers.append(ghost_marker)

        hud_lines = [
            f"score={game.get('score', 0)}",
            f"time={game.get('time_remaining', 0.0):.1f}s",
            f"target={auto.get('target_id', '-')}",
            f"collected={auto.get('collected', 0)}",
            f"remaining={auto.get('remaining', game.get('pellets_remaining', 0))}",
            f"mode={game.get('mode', '')}",
        ]
        if auto.get("complete", False) or game.get("victory", False):
            hud_lines.append("VICTORY")
        elif game.get("game_over", False):
            hud_lines.append("GAME OVER")

        for offset, text in enumerate(hud_lines):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = stamp
            marker.ns = "hud"
            marker.id = 1000 + offset
            marker.action = Marker.ADD
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.position.x = PACMAN_WORLD_BOUNDS.x_min + 1.0
            marker.pose.position.y = PACMAN_WORLD_BOUNDS.y_max - 0.8 - 0.45 * offset
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.28
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = text
            markers.markers.append(marker)

        self.marker_pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
