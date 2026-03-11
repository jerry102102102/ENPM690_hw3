#!/usr/bin/env python3

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from simulation_interfaces.srv import GetEntities, SetEntityState, SpawnEntity

from .gazebo_fish_utils import (
    fish_model_paths,
    make_get_entities_request,
    make_set_entity_state_request,
    make_spawn_entity_request,
)


class GazeboFishSync(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_fish_sync")
        self.declare_parameter("fish_state_topic", "/phase2/fish_state_json")
        self.declare_parameter("startup_timeout", 30.0)

        self.fish_state_topic = str(self.get_parameter("fish_state_topic").value)
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self.model_paths = fish_model_paths()
        self.known_entities: set[str] = set()
        self._services_ready = False
        self._pending_fish_list: list[dict] = []
        self._last_wait_log_time = 0.0

        self.spawn_entity_client = self.create_client(SpawnEntity, "/gzserver/spawn_entity")
        self.get_entities_client = self.create_client(GetEntities, "/gzserver/get_entities")
        self.set_entity_state_client = self.create_client(SetEntityState, "/gzserver/set_entity_state")

        self.create_subscription(String, self.fish_state_topic, self._fish_state_callback, 10)
        self.create_timer(0.5, self._startup_tick)
        self.get_logger().info(f"gazebo fish sync listening on {self.fish_state_topic}")

    def _startup_tick(self) -> None:
        if self._services_ready:
            return

        services = (
            self.spawn_entity_client,
            self.get_entities_client,
            self.set_entity_state_client,
        )
        ready = all(client.wait_for_service(timeout_sec=0.01) for client in services)
        now = time.time()
        if not ready:
            if now - self._last_wait_log_time >= 5.0:
                self.get_logger().info("waiting for Gazebo fish sync services...")
                self._last_wait_log_time = now
            return

        self._services_ready = True
        self.get_logger().info("Gazebo fish sync services are ready")
        if self._pending_fish_list:
            self._ensure_fish_entities(self._pending_fish_list)
            for fish in self._pending_fish_list:
                self._sync_single_fish(fish)

    def _call_service(self, client, request, timeout_sec: float = 5.0):
        future = client.call_async(request)
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                result = future.result()
                if result is None:
                    raise RuntimeError(f"Service call failed for {client.srv_name}")
                return result
            time.sleep(0.01)
        raise TimeoutError(f"Timed out calling service {client.srv_name}")

    def _fish_state_callback(self, msg: String) -> None:
        try:
            fish_list = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"failed to parse /phase2/fish_state_json: {exc}")
            return

        if not isinstance(fish_list, list):
            self.get_logger().warning("fish state payload is not a list")
            return

        self._pending_fish_list = fish_list
        if not self._services_ready:
            return
        self._ensure_fish_entities(fish_list)
        for fish in fish_list:
            self._sync_single_fish(fish)

    def _refresh_known_entities(self) -> None:
        response = self._call_service(self.get_entities_client, make_get_entities_request())
        self.known_entities = set(response.entities)

    def _ensure_fish_entities(self, fish_list: list[dict]) -> None:
        unseen = [fish for fish in fish_list if fish.get("fish_id") not in self.known_entities]
        if unseen:
            self._refresh_known_entities()

        for fish in fish_list:
            fish_id = fish.get("fish_id")
            species = fish.get("species")
            if not fish_id or not species:
                continue
            if fish_id in self.known_entities:
                continue
            request = make_spawn_entity_request(
                entity_name=fish_id,
                model_path=self.model_paths[species],
                x=float(fish.get("x", 0.0)),
                y=float(fish.get("y", 0.0)),
            )
            self._call_service(self.spawn_entity_client, request)
            self.known_entities.add(fish_id)
            self.get_logger().info(f"spawned Gazebo fish entity {fish_id}")

    def _sync_single_fish(self, fish: dict) -> None:
        fish_id = fish.get("fish_id")
        species = fish.get("species")
        if not fish_id or not species:
            return

        request = make_set_entity_state_request(
            entity_name=fish_id,
            x=float(fish.get("x", 0.0)),
            y=float(fish.get("y", 0.0)),
            active=bool(fish.get("active", False)),
        )
        try:
            self._call_service(self.set_entity_state_client, request)
        except Exception as exc:
            self.get_logger().warning(f"set state failed for {fish_id}, attempting respawn: {exc}")
            self._refresh_known_entities()
            if fish_id not in self.known_entities:
                spawn_request = make_spawn_entity_request(
                    entity_name=fish_id,
                    model_path=self.model_paths[species],
                    x=float(fish.get('x', 0.0)),
                    y=float(fish.get('y', 0.0)),
                )
                self._call_service(self.spawn_entity_client, spawn_request)
                self.known_entities.add(fish_id)
            self._call_service(self.set_entity_state_client, request)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = GazeboFishSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
