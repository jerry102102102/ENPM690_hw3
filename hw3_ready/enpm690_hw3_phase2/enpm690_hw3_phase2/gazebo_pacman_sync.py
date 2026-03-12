#!/usr/bin/env python3

from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from simulation_interfaces.srv import DeleteEntity, SetEntityState, SpawnEntity

from .gazebo_entity_utils import (
    make_delete_entity_request,
    make_set_entity_state_request,
    make_spawn_entity_request,
)


class GazeboPacmanSync(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_pacman_sync")
        self.declare_parameter("pellet_state_topic", "/phase2/pellet_state_json")
        self.declare_parameter("ghost_state_topic", "/phase2/ghost_state_json")
        self.declare_parameter("service_timeout", 10.0)
        self.declare_parameter("spawn_missing_entities", False)

        share = Path(get_package_share_directory("enpm690_hw3_phase2"))
        self.pellet_model = share / "models" / "pellet_simple" / "model.sdf"
        self.ghost_model = share / "models" / "ghost_simple" / "model.sdf"
        self.service_timeout = float(self.get_parameter("service_timeout").value)
        self.spawn_missing_entities = bool(self.get_parameter("spawn_missing_entities").value)

        self.spawn_client = self.create_client(SpawnEntity, "/gzserver/spawn_entity")
        self.delete_client = self.create_client(DeleteEntity, "/gzserver/delete_entity")
        self.set_state_client = self.create_client(SetEntityState, "/gzserver/set_entity_state")

        self.pellet_payload = "[]"
        self.ghost_payload = "{}"
        self.spawned_entities: set[str] = set()
        self.deleted_pellets: set[str] = set()
        self.ghost_spawned = False

        self.create_subscription(String, str(self.get_parameter("pellet_state_topic").value), self._pellet_callback, 10)
        self.create_subscription(String, str(self.get_parameter("ghost_state_topic").value), self._ghost_callback, 10)
        self.create_timer(0.2, self._tick)

    def _pellet_callback(self, msg: String) -> None:
        self.pellet_payload = msg.data

    def _ghost_callback(self, msg: String) -> None:
        self.ghost_payload = msg.data

    def _tick(self) -> None:
        if not all(client.wait_for_service(timeout_sec=0.01) for client in (self.spawn_client, self.delete_client, self.set_state_client)):
            return

        pellets = json.loads(self.pellet_payload or "[]")
        ghost = json.loads(self.ghost_payload or "{}")
        self._sync_pellets(pellets)
        self._sync_ghost(ghost)

    def _sync_pellets(self, pellets: list[dict]) -> None:
        for pellet in pellets:
            pellet_id = str(pellet.get("pellet_id", ""))
            if not pellet_id:
                continue
            active = bool(pellet.get("active", False))
            if active and pellet_id not in self.spawned_entities and self.spawn_missing_entities:
                request = make_spawn_entity_request(
                    pellet_id,
                    self.pellet_model,
                    float(pellet.get("x", 0.0)),
                    float(pellet.get("y", 0.0)),
                    z=0.06,
                )
                self._call(self.spawn_client, request)
                self.spawned_entities.add(pellet_id)
                self.get_logger().info(f"[GZ_SYNC] spawned pellet {pellet_id}")
            elif not active and pellet_id not in self.deleted_pellets:
                self._call(self.delete_client, make_delete_entity_request(pellet_id))
                self.deleted_pellets.add(pellet_id)
                self.spawned_entities.discard(pellet_id)
                self.get_logger().info(f"[GZ_SYNC] deleted pellet {pellet_id}")

    def _sync_ghost(self, ghost: dict) -> None:
        ghost_id = str(ghost.get("ghost_id", ""))
        if not ghost_id:
            return
        if not self.ghost_spawned and self.spawn_missing_entities:
            request = make_spawn_entity_request(
                ghost_id,
                self.ghost_model,
                float(ghost.get("x", 0.0)),
                float(ghost.get("y", 0.0)),
                z=0.18,
            )
            self._call(self.spawn_client, request)
            self.ghost_spawned = True
            self.spawned_entities.add(ghost_id)
            self.get_logger().info(f"[GZ_SYNC] spawned ghost {ghost_id}")
            return

        self.ghost_spawned = True
        request = make_set_entity_state_request(
            ghost_id,
            float(ghost.get("x", 0.0)),
            float(ghost.get("y", 0.0)),
            float(ghost.get("heading", 0.0)),
            z=0.18,
        )
        self._call(self.set_state_client, request)

    def _call(self, client, request):
        future = client.call_async(request)
        deadline = time.time() + self.service_timeout
        while time.time() < deadline:
            if future.done():
                result = future.result()
                if result is None:
                    raise RuntimeError(f"Service call failed for {client.srv_name}")
                return result
            time.sleep(0.01)
        raise TimeoutError(f"Timed out calling service {client.srv_name}")


def main() -> None:
    rclpy.init()
    node = GazeboPacmanSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
