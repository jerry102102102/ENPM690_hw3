#!/usr/bin/env python3

from __future__ import annotations

import importlib
import json
import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from simulation_interfaces.srv import SetEntityState

from .gazebo_fish_utils import (
    fish_model_paths,
    make_set_entity_state_request,
)


class GazeboFishSync(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_fish_sync")
        self.declare_parameter("fish_state_topic", "/phase2/fish_state_json")
        self.declare_parameter("service_timeout", 15.0)
        self.declare_parameter("sync_period", 0.1)
        self.declare_parameter("moving_sync_distance", 0.015)
        self.declare_parameter("static_sync_distance", 0.05)
        self.declare_parameter("max_sync_staleness", 0.5)
        self.declare_parameter("ready_topic", "/phase2/fish_sync_ready")
        self.declare_parameter("world_name", "phase2_ocean_arena")

        self.fish_state_topic = str(self.get_parameter("fish_state_topic").value)
        self.service_timeout = float(self.get_parameter("service_timeout").value)
        self.sync_period = float(self.get_parameter("sync_period").value)
        self.moving_sync_distance = float(self.get_parameter("moving_sync_distance").value)
        self.static_sync_distance = float(self.get_parameter("static_sync_distance").value)
        self.max_sync_staleness = float(self.get_parameter("max_sync_staleness").value)
        self.ready_topic = str(self.get_parameter("ready_topic").value)
        self.world_name = str(self.get_parameter("world_name").value)
        self.model_paths = fish_model_paths()
        self.known_entities: set[str] = set()
        self.last_synced_state: dict[str, tuple[float, float, bool, float]] = {}
        self._services_ready = False
        self._pending_fish_list: list[dict] = []
        self._last_wait_log_time = 0.0
        self._last_error_log_time = 0.0
        self._last_ready_state = False
        self._callback_group = ReentrantCallbackGroup()
        self._gz_pose_batch = self._init_gz_pose_batch_client()

        self.ready_pub = self.create_publisher(Bool, self.ready_topic, 10)
        self.set_entity_state_client = None
        if self._gz_pose_batch is None:
            self.set_entity_state_client = self.create_client(
                SetEntityState,
                "/gzserver/set_entity_state",
                callback_group=self._callback_group,
            )
            self.get_logger().warning(
                "gz transport Pose_V bindings unavailable, falling back to per-entity /gzserver/set_entity_state"
            )
        else:
            self.get_logger().info(
                f"using Gazebo batch pose service /world/{self.world_name}/set_pose_vector"
            )

        self.create_subscription(String, self.fish_state_topic, self._fish_state_callback, 10, callback_group=self._callback_group)
        self.create_timer(self.sync_period, self._sync_tick, callback_group=self._callback_group)
        self.get_logger().info(f"gazebo fish sync listening on {self.fish_state_topic}")

    def _sync_tick(self) -> None:
        ready = True
        if self.set_entity_state_client is not None:
            services = (self.set_entity_state_client,)
            ready = all(client.wait_for_service(timeout_sec=0.01) for client in services)
        now = time.time()
        if not ready:
            self._services_ready = False
            self._publish_ready(False)
            if now - self._last_wait_log_time >= 5.0:
                self.get_logger().info("waiting for Gazebo fish sync services...")
                self._last_wait_log_time = now
            return

        if not self._services_ready:
            self._services_ready = True
            self.get_logger().info("Gazebo fish sync services are ready")

        if not self._pending_fish_list:
            return

        try:
            self._ensure_fish_entities(self._pending_fish_list)
            self._sync_known_fish(self._pending_fish_list)
            self._publish_ready(self._all_active_fish_present())
        except Exception as exc:
            self._services_ready = False
            self._publish_ready(False)
            self._log_sync_warning(f"fish sync failed, will retry: {exc}")

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
            self._publish_ready(False)

    def _ensure_fish_entities(self, fish_list: list[dict]) -> None:
        self.known_entities = {
            str(fish.get("fish_id"))
            for fish in fish_list
            if fish.get("fish_id")
        }

    def _sync_known_fish(self, fish_list: list[dict]) -> None:
        pending = []
        for fish in fish_list:
            fish_id = fish.get("fish_id")
            if fish_id not in self.known_entities:
                continue
            if not self._should_sync_fish(fish):
                continue
            pending.append(fish)

        if not pending:
            return

        if self._gz_pose_batch is not None:
            self._sync_batch_pose_vector(pending)
            return

        for fish in pending:
            self._sync_single_fish(fish)

    def _all_active_fish_present(self) -> bool:
        fish_ids = {
            str(fish.get("fish_id"))
            for fish in self._pending_fish_list
            if fish.get("fish_id")
        }
        return bool(fish_ids) and fish_ids.issubset(self.known_entities)

    def _should_sync_fish(self, fish: dict) -> bool:
        fish_id = fish.get("fish_id")
        if not fish_id:
            return False

        x = float(fish.get("x", 0.0))
        y = float(fish.get("y", 0.0))
        active = bool(fish.get("active", False))
        species = str(fish.get("species", ""))
        now = time.time()

        previous = self.last_synced_state.get(fish_id)
        if previous is None:
            return True

        prev_x, prev_y, prev_active, prev_time = previous
        if active != prev_active:
            return True

        distance = math.hypot(x - prev_x, y - prev_y)
        threshold = self.static_sync_distance if species == "seaweed" else self.moving_sync_distance
        if distance >= threshold:
            return True

        return now - prev_time >= self.max_sync_staleness

    def _sync_single_fish(self, fish: dict) -> None:
        fish_id = fish.get("fish_id")
        if not fish_id:
            return

        request = make_set_entity_state_request(
            entity_name=fish_id,
            x=float(fish.get("x", 0.0)),
            y=float(fish.get("y", 0.0)),
            active=bool(fish.get("active", False)),
        )
        try:
            self._call_service(self.set_entity_state_client, request, timeout_sec=self.service_timeout)
            self.last_synced_state[fish_id] = (
                float(fish.get("x", 0.0)),
                float(fish.get("y", 0.0)),
                bool(fish.get("active", False)),
                time.time(),
            )
        except Exception as exc:
            self.known_entities.discard(fish_id)
            self.last_synced_state.pop(fish_id, None)
            self._log_sync_warning(f"set state failed for {fish_id}, will respawn later: {exc}")

    def _sync_batch_pose_vector(self, fish_list: list[dict]) -> None:
        request = self._gz_pose_batch["pose_v_cls"]()
        now = time.time()

        for fish in fish_list:
            pose = request.pose.add()
            pose.name = str(fish["fish_id"])
            pose.position.x = float(fish.get("x", 0.0))
            pose.position.y = float(fish.get("y", 0.0))
            pose.position.z = 0.15 if bool(fish.get("active", False)) else -2.0
            pose.orientation.w = 1.0

        ok, response = self._gz_pose_batch["node"].request(
            self._gz_pose_batch["service_name"],
            request,
            self._gz_pose_batch["pose_v_cls"],
            self._gz_pose_batch["boolean_cls"],
            int(self.service_timeout * 1000.0),
        )
        if not ok:
            raise TimeoutError(f"Timed out calling Gazebo batch pose service {self._gz_pose_batch['service_name']}")
        if hasattr(response, "data") and not response.data:
            raise RuntimeError(f"Gazebo batch pose service rejected update on {self._gz_pose_batch['service_name']}")

        for fish in fish_list:
            self.last_synced_state[str(fish["fish_id"])] = (
                float(fish.get("x", 0.0)),
                float(fish.get("y", 0.0)),
                bool(fish.get("active", False)),
                now,
            )

    def _log_sync_warning(self, message: str) -> None:
        now = time.time()
        if now - self._last_error_log_time >= 1.0:
            self.get_logger().warning(message)
            self._last_error_log_time = now

    def _publish_ready(self, ready: bool) -> None:
        if ready == self._last_ready_state:
            return
        self.ready_pub.publish(Bool(data=ready))
        self._last_ready_state = ready

    def _init_gz_pose_batch_client(self) -> dict | None:
        candidates = [
            ("gz.transport", "gz.msgs.pose_v_pb2", "gz.msgs.boolean_pb2"),
            ("gz.transport15", "gz.msgs.pose_v_pb2", "gz.msgs.boolean_pb2"),
            ("gz.transport14", "gz.msgs11.pose_v_pb2", "gz.msgs11.boolean_pb2"),
            ("gz.transport13", "gz.msgs10.pose_v_pb2", "gz.msgs10.boolean_pb2"),
        ]
        for transport_mod, pose_mod, boolean_mod in candidates:
            try:
                node_cls = getattr(importlib.import_module(transport_mod), "Node")
                pose_v_cls = getattr(importlib.import_module(pose_mod), "Pose_V")
                boolean_cls = getattr(importlib.import_module(boolean_mod), "Boolean")
                return {
                    "node": node_cls(),
                    "pose_v_cls": pose_v_cls,
                    "boolean_cls": boolean_cls,
                    "service_name": f"/world/{self.world_name}/set_pose_vector",
                }
            except Exception:
                continue
        return None


def main() -> None:
    rclpy.init()
    node = None
    executor = None
    try:
        node = GazeboFishSync()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
