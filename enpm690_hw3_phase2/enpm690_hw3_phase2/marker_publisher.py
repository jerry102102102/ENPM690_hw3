#!/usr/bin/env python3

from __future__ import annotations

from typing import Any

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .constants import PHASE2_WORLD_BOUNDS, SPECIES_CONFIGS
from .spawn_utils import fish_states_from_json, game_snapshot_from_json


class MarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("marker_publisher")
        self.declare_parameter("marker_topic", "/phase2/markers")
        self.marker_topic = str(self.get_parameter("marker_topic").value)

        self.fish_payload = ""
        self.game_payload = ""
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(String, "/phase2/fish_state_json", self._fish_callback, 10)
        self.create_subscription(String, "/phase2/game_state_json", self._game_callback, 10)
        self.create_timer(0.1, self._tick)

    def _fish_callback(self, msg: String) -> None:
        self.fish_payload = msg.data

    def _game_callback(self, msg: String) -> None:
        self.game_payload = msg.data

    def _tick(self) -> None:
        fish_states = fish_states_from_json(self.fish_payload)
        game_state = game_snapshot_from_json(self.game_payload)
        markers = MarkerArray()

        for index, fish in enumerate(fish_states):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = fish["species"]
            marker.id = index
            marker.action = Marker.ADD if fish.get("active", False) else Marker.DELETE
            marker.type = Marker.CYLINDER if fish["species"] == "seaweed" else Marker.SPHERE
            marker.pose.position.x = float(fish["x"])
            marker.pose.position.y = float(fish["y"])
            marker.pose.position.z = 0.20
            marker.pose.orientation.w = 1.0
            marker.scale.x = fish["radius"] * 2.0
            marker.scale.y = fish["radius"] * 2.0
            marker.scale.z = 0.35 if fish["species"] == "seaweed" else fish["radius"] * 2.0
            rgba = SPECIES_CONFIGS[fish["species"]].color_rgba
            marker.color.r = rgba[0]
            marker.color.g = rgba[1]
            marker.color.b = rgba[2]
            marker.color.a = rgba[3]
            markers.markers.append(marker)

        text_lines = [
            f"score={game_state.get('score', 0)}",
            f"time={game_state.get('time_remaining', 0.0):.1f}s",
            f"mode={game_state.get('mode', '')}",
            f"target={game_state.get('current_target_id', '')}",
        ]
        for offset, text in enumerate(text_lines):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "hud"
            marker.id = 1000 + offset
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = PHASE2_WORLD_BOUNDS.x_min + 0.8
            marker.pose.position.y = PHASE2_WORLD_BOUNDS.y_max - 0.6 - 0.35 * offset
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.22
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
