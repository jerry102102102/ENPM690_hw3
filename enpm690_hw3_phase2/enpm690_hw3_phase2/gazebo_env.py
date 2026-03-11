from __future__ import annotations

import math
import random
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, TextIO

import gymnasium as gym
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from gymnasium import spaces
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from simulation_interfaces.msg import EntityFilters, EntityState, Resource, SimulationState
from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetSimulationState,
    ResetSimulation,
    SetEntityState,
    SetSimulationState,
    SpawnEntity,
    StepSimulation,
)

from .constants import (
    DEFAULT_SHARK_SPAWN,
    GAME_DT,
    GAME_DURATION_SECONDS,
    LIDAR_BIN_COUNT,
    PHASE2_OBSTACLES,
    PHASE2_WORLD_BOUNDS,
    SHARK_MAX_ANGULAR_SPEED,
    SHARK_MAX_LINEAR_SPEED,
    SHARK_RADIUS,
    SHARK_STUN_SECONDS,
    SharkState,
)
from .fish_manager import FishManager
from .geometry_utils import angle_diff, bearing_xy, distance_xy
from .observation_builder import ObservationBuilder
from .reward_builder import RewardBuilder
from .shark_collision_monitor import SharkCollisionMonitor


class _GazeboTrainingNode(Node):
    def __init__(self, robot_name: str, command_topic: str = "/cmd_vel") -> None:
        super().__init__("gazebo_shark_hunt_env")
        self.robot_name = robot_name
        self.command_topic = command_topic
        self.latest_scan: LaserScan | None = None
        self.latest_odom: Odometry | None = None
        self.latest_clock: Clock | None = None
        self.scan_count = 0
        self.odom_count = 0
        self.clock_count = 0

        self.cmd_pub = self.create_publisher(Twist, self.command_topic, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 20)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(Clock, "/clock", self._clock_callback, 20)

        self.reset_client = self.create_client(ResetSimulation, "/gzserver/reset_simulation")
        self.step_client = self.create_client(StepSimulation, "/gzserver/step_simulation")
        self.get_sim_state_client = self.create_client(GetSimulationState, "/gzserver/get_simulation_state")
        self.set_sim_state_client = self.create_client(SetSimulationState, "/gzserver/set_simulation_state")
        self.spawn_entity_client = self.create_client(SpawnEntity, "/gzserver/spawn_entity")
        self.delete_entity_client = self.create_client(DeleteEntity, "/gzserver/delete_entity")
        self.get_entities_client = self.create_client(GetEntities, "/gzserver/get_entities")
        self.set_entity_state_client = self.create_client(SetEntityState, "/gzserver/set_entity_state")

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.scan_count += 1

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_count += 1

    def _clock_callback(self, msg: Clock) -> None:
        self.latest_clock = msg
        self.clock_count += 1


class GazeboSharkHuntEnv(gym.Env[np.ndarray, np.ndarray]):
    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        launch_stack: bool = False,
        headless: bool = True,
        stack_timeout: float = 45.0,
        enable_fish_visuals: bool = True,
        robot_name: str | None = None,
        command_topic: str = "/cmd_vel",
        launch_log_mode: str = "silent",
    ) -> None:
        super().__init__()
        self.launch_stack = launch_stack
        self.headless = headless
        self.stack_timeout = stack_timeout
        self.enable_fish_visuals = enable_fish_visuals
        self.robot_name = robot_name or ("tb3_phase2_train" if headless else "tb3_phase2_eval")
        self.command_topic = command_topic
        self.launch_log_mode = launch_log_mode

        low = np.array(
            [0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0]
            + [0.0] * LIDAR_BIN_COUNT
            + [0.0, -1.0, -1.0] * 3,
            dtype=np.float32,
        )
        high = np.array(
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            + [1.0] * LIDAR_BIN_COUNT
            + [1.0, 1.0, 1.0] * 3,
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.dt = GAME_DT
        self.max_steps = int(GAME_DURATION_SECONDS / self.dt)
        self.physics_steps_per_action = 10
        self.rng = random.Random(690)
        self.fish_manager = FishManager(PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES, self.rng)
        self.collision_monitor = SharkCollisionMonitor(PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES, SHARK_RADIUS)
        self.reward_builder = RewardBuilder()
        self.shark = SharkState()
        self.steps = 0
        self.score = 0
        self.catch_counts = {"tuna": 0, "sardine": 0, "seaweed": 0}
        self.time_remaining = GAME_DURATION_SECONDS

        self.launch_process: subprocess.Popen[str] | None = None
        self.launch_log_handle: TextIO | None = None
        if self.launch_stack:
            self.launch_process = self._launch_stack_process()

        self._rclpy_owner = False
        if not rclpy.ok():
            rclpy.init()
            self._rclpy_owner = True

        self.node = _GazeboTrainingNode(self.robot_name, self.command_topic)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.fish_model_paths = self._fish_model_paths()
        self.entity_name_map: dict[str, str] = {}
        self._wait_for_stack_ready()
        self._set_paused_state()
        self._ensure_fish_entities()

    def reset(self, *, seed: int | None = None, options: dict | None = None) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        if seed is not None:
            self.rng.seed(seed)

        self._set_paused_state()
        reset_request = ResetSimulation.Request()
        reset_request.scope = ResetSimulation.Request.SCOPE_TIME | ResetSimulation.Request.SCOPE_STATE
        self._call_service(self.node.reset_client, reset_request, timeout_sec=self.stack_timeout)

        self.shark = SharkState(x=DEFAULT_SHARK_SPAWN[0], y=DEFAULT_SHARK_SPAWN[1], heading=DEFAULT_SHARK_SPAWN[2])
        self.steps = 0
        self.score = 0
        self.time_remaining = GAME_DURATION_SECONDS
        self.catch_counts = {"tuna": 0, "sardine": 0, "seaweed": 0}
        self.fish_manager.reset()

        self._publish_zero_command()
        self._set_robot_state(self.shark)
        self._ensure_fish_entities()
        self._sync_all_fish_to_gazebo()
        self._step_and_wait(self.physics_steps_per_action)
        self._update_shark_from_odom()
        observation = self._build_observation()
        info = self._build_info(collision=False, reward_components={})
        info["score"] = self.score
        info["catch_counts"] = dict(self.catch_counts)
        return observation, info

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)
        target_before = self._current_target()
        distance_before = self._distance_to_target(target_before)

        cmd = Twist()
        if self.shark.collision_cooldown > 0.0:
            self.shark.collision_cooldown = max(0.0, self.shark.collision_cooldown - self.dt)
        else:
            cmd.linear.x = 0.5 * (float(action[0]) + 1.0) * SHARK_MAX_LINEAR_SPEED
            cmd.angular.z = float(action[1]) * SHARK_MAX_ANGULAR_SPEED

        self.node.cmd_pub.publish(cmd)
        self._step_and_wait(self.physics_steps_per_action)
        self._update_shark_from_odom()

        collision = False
        if self.shark.collision_cooldown <= 0.0 and self.collision_monitor.check_collision(self.shark):
            collision = True
            self.shark.collision_cooldown = SHARK_STUN_SECONDS
            self._publish_zero_command()
            self._set_robot_state(self.shark, zero_twist=True)

        self.fish_manager.update(self.dt, self.shark, immediate_respawn=True)
        catches = self.fish_manager.detect_catches(self.shark, immediate_respawn=True)
        catch_score = sum(event.score_delta for event in catches)
        for event in catches:
            self.score += event.score_delta
            self.catch_counts[event.species] += 1

        self._sync_all_fish_to_gazebo()

        self.steps += 1
        self.time_remaining = max(0.0, GAME_DURATION_SECONDS - self.steps * self.dt)
        distance_after = self._distance_to_target(target_before)
        lidar = self._latest_lidar_proximity()
        _, front_prox, _ = ObservationBuilder.front_sector_proximities(lidar)
        reward, components = self.reward_builder.compute(
            catch_score=catch_score,
            collision=collision,
            distance_before=distance_before,
            distance_after=distance_after,
            front_obstacle_proximity=front_prox,
        )

        observation = self._build_observation(lidar)
        terminated = False
        truncated = self.steps >= self.max_steps
        info = self._build_info(collision=collision, reward_components=components)
        info["score"] = self.score
        info["catch_counts"] = dict(self.catch_counts)
        return observation, float(reward), terminated, truncated, info

    def close(self) -> None:
        try:
            self._publish_zero_command()
        except Exception:
            pass

        if hasattr(self, "executor"):
            self.executor.shutdown()
        if hasattr(self, "node"):
            self.node.destroy_node()
        if hasattr(self, "spin_thread") and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        if self.launch_process is not None:
            self.launch_process.terminate()
            try:
                self.launch_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.launch_process.kill()
        if self.launch_log_handle is not None:
            self.launch_log_handle.close()
        if self._rclpy_owner and rclpy.ok():
            rclpy.shutdown()

    def _launch_stack_process(self) -> subprocess.Popen[str]:
        launch_file = "phase2_train.launch.py" if self.headless else "phase2_eval.launch.py"
        stdout_target: int | TextIO
        stderr_target: int | TextIO
        if self.launch_log_mode == "inherit":
            stdout_target = None  # type: ignore[assignment]
            stderr_target = None  # type: ignore[assignment]
        elif self.launch_log_mode == "file":
            log_dir = Path("artifacts/phase2_stack_logs")
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"{launch_file.replace('.launch.py', '')}.log"
            self.launch_log_handle = log_path.open("a")
            stdout_target = self.launch_log_handle
            stderr_target = self.launch_log_handle
        else:
            stdout_target = subprocess.DEVNULL
            stderr_target = subprocess.DEVNULL
        return subprocess.Popen(
            ["ros2", "launch", "enpm690_hw3_phase2", launch_file],
            stdout=stdout_target,
            stderr=stderr_target,
            text=True,
        )

    def _wait_for_stack_ready(self) -> None:
        deadline = time.time() + self.stack_timeout
        clients = [
            self.node.reset_client,
            self.node.step_client,
            self.node.get_sim_state_client,
            self.node.set_sim_state_client,
            self.node.spawn_entity_client,
            self.node.delete_entity_client,
            self.node.get_entities_client,
            self.node.set_entity_state_client,
        ]
        for client in clients:
            remaining = max(0.1, deadline - time.time())
            if not client.wait_for_service(timeout_sec=remaining):
                raise RuntimeError(f"Required Gazebo simulation service not ready: {client.srv_name}")

        while time.time() < deadline:
            if self.node.latest_scan is not None and self.node.latest_odom is not None:
                self._call_service(self.node.get_sim_state_client, GetSimulationState.Request(), timeout_sec=5.0)
                if self._command_topic_has_subscriber():
                    return
            time.sleep(0.1)
        if self.node.latest_scan is None or self.node.latest_odom is None:
            raise RuntimeError("Gazebo training stack is not ready: missing /scan or /odom.")
        raise RuntimeError(
            f"Gazebo training stack is ready for sensors/services, but the shark command topic "
            f"{self.command_topic} has no subscriber. Check whether training should publish to /cmd_vel "
            f"or whether a command-gating node is missing."
        )

    def _command_topic_has_subscriber(self) -> bool:
        topic_names_and_types = dict(self.node.get_topic_names_and_types())
        if self.command_topic not in topic_names_and_types:
            return False
        return self.node.count_subscribers(self.command_topic) > 0

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

    def _set_paused_state(self) -> None:
        request = SetSimulationState.Request()
        request.state = SimulationState()
        request.state.state = SimulationState.STATE_PAUSED
        self._call_service(self.node.set_sim_state_client, request, timeout_sec=self.stack_timeout)

    def _step_and_wait(self, steps: int) -> None:
        previous_scan_count = self.node.scan_count
        previous_odom_count = self.node.odom_count
        request = StepSimulation.Request()
        request.steps = steps
        self._call_service(self.node.step_client, request, timeout_sec=self.stack_timeout)

        deadline = time.time() + self.stack_timeout
        while time.time() < deadline:
            if self.node.scan_count > previous_scan_count and self.node.odom_count > previous_odom_count:
                return
            time.sleep(0.01)
        raise TimeoutError("Timed out waiting for fresh /scan and /odom after stepping Gazebo.")

    def _update_shark_from_odom(self) -> None:
        odom = self.node.latest_odom
        if odom is None:
            raise RuntimeError("No /odom message available.")
        self.shark.x = odom.pose.pose.position.x
        self.shark.y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.shark.heading = math.atan2(siny_cosp, cosy_cosp)
        self.shark.linear_speed = odom.twist.twist.linear.x
        self.shark.angular_speed = odom.twist.twist.angular.z

    def _latest_lidar_proximity(self) -> np.ndarray:
        scan = self.node.latest_scan
        if scan is None:
            raise RuntimeError("No /scan message available.")
        return ObservationBuilder.preprocess_scan_ranges(scan.ranges, scan.range_min, scan.range_max, LIDAR_BIN_COUNT)

    def _build_observation(self, lidar: np.ndarray | None = None) -> np.ndarray:
        if lidar is None:
            lidar = self._latest_lidar_proximity()
        return ObservationBuilder.build_vector(self.shark, self.time_remaining, lidar, self.fish_manager.fish)

    def _current_target(self):
        best = None
        best_utility = -1.0
        for species, score in (("tuna", 10.0), ("sardine", 3.0), ("seaweed", 1.0)):
            candidate = self.fish_manager.nearest_active_by_species(species, self.shark)
            if candidate is None:
                continue
            utility = score / (distance_xy(self.shark.x, self.shark.y, candidate.x, candidate.y) + 1e-3)
            if utility > best_utility:
                best = candidate
                best_utility = utility
        return best

    def _distance_to_target(self, target) -> float | None:
        if target is None:
            return None
        return distance_xy(self.shark.x, self.shark.y, target.x, target.y)

    def _build_info(self, collision: bool, reward_components: dict[str, float]) -> dict[str, Any]:
        target = self._current_target()
        if target is None:
            target_id = ""
            target_species = ""
            target_distance = None
            target_bearing = None
        else:
            target_id = target.fish_id
            target_species = target.species
            target_distance = distance_xy(self.shark.x, self.shark.y, target.x, target.y)
            target_bearing = angle_diff(bearing_xy(self.shark.x, self.shark.y, target.x, target.y), self.shark.heading)
        return {
            "collision": collision,
            "reward_components": reward_components,
            "target_id": target_id,
            "target_species": target_species,
            "target_distance": target_distance,
            "target_bearing": target_bearing,
            "time_remaining": self.time_remaining,
        }

    def _fish_model_paths(self) -> dict[str, Path]:
        share = Path(get_package_share_directory("enpm690_hw3_phase2"))
        return {
            "tuna": share / "models" / "tuna_simple" / "model.sdf",
            "sardine": share / "models" / "sardine_simple" / "model.sdf",
            "seaweed": share / "models" / "seaweed_simple" / "model.sdf",
        }

    def _get_entity_names(self) -> set[str]:
        request = GetEntities.Request()
        request.filters = EntityFilters()
        request.filters.filter = ""
        response = self._call_service(self.node.get_entities_client, request, timeout_sec=self.stack_timeout)
        return set(response.entities)

    def _ensure_fish_entities(self) -> None:
        if not self.enable_fish_visuals:
            return
        existing = self._get_entity_names()
        for fish in self.fish_manager.fish:
            if fish.fish_id in existing:
                self.entity_name_map[fish.fish_id] = fish.fish_id
                continue
            request = SpawnEntity.Request()
            request.name = fish.fish_id
            request.allow_renaming = False
            request.entity_resource = Resource(uri=self.fish_model_paths[fish.species].as_uri(), resource_string="")
            request.initial_pose = PoseStamped()
            request.initial_pose.header.frame_id = "world"
            request.initial_pose.pose.position.x = fish.x
            request.initial_pose.pose.position.y = fish.y
            request.initial_pose.pose.position.z = 0.15
            request.initial_pose.pose.orientation.w = 1.0
            response = self._call_service(self.node.spawn_entity_client, request, timeout_sec=self.stack_timeout)
            entity_name = getattr(response, "entity_name", fish.fish_id) or fish.fish_id
            self.entity_name_map[fish.fish_id] = entity_name

    def _sync_all_fish_to_gazebo(self) -> None:
        if not self.enable_fish_visuals:
            return
        for fish in self.fish_manager.fish:
            self._set_fish_entity_state(fish)

    def _set_fish_entity_state(self, fish) -> None:
        entity_name = self.entity_name_map.get(fish.fish_id, fish.fish_id)
        request = SetEntityState.Request()
        request.entity = entity_name
        request.state = EntityState()
        request.state.header.frame_id = "world"
        request.state.pose.position.x = fish.x
        request.state.pose.position.y = fish.y
        request.state.pose.position.z = -2.0 if not fish.active else 0.15
        request.state.pose.orientation.w = 1.0
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
        request.set_pose = True
        request.set_twist = True
        request.set_acceleration = False
        self._call_service(self.node.set_entity_state_client, request, timeout_sec=self.stack_timeout)

    def _set_robot_state(self, shark: SharkState, zero_twist: bool = True) -> None:
        request = SetEntityState.Request()
        request.entity = self.robot_name
        request.state = EntityState()
        request.state.header.frame_id = "world"
        request.state.pose.position.x = shark.x
        request.state.pose.position.y = shark.y
        request.state.pose.position.z = 0.08
        request.state.pose.orientation.z = math.sin(shark.heading / 2.0)
        request.state.pose.orientation.w = math.cos(shark.heading / 2.0)
        if zero_twist:
            request.state.twist.linear.x = 0.0
            request.state.twist.linear.y = 0.0
            request.state.twist.linear.z = 0.0
            request.state.twist.angular.x = 0.0
            request.state.twist.angular.y = 0.0
            request.state.twist.angular.z = 0.0
        request.set_pose = True
        request.set_twist = True
        request.set_acceleration = False
        self._call_service(self.node.set_entity_state_client, request, timeout_sec=self.stack_timeout)

    def _publish_zero_command(self) -> None:
        self.node.cmd_pub.publish(Twist())

    def run_motion_smoke_test(self, steps: int = 5, action: np.ndarray | None = None) -> None:
        if action is None:
            action = np.array([-0.4, 0.0], dtype=np.float32)
        observation, _ = self.reset()
        start_x = self.shark.x
        start_y = self.shark.y
        for _ in range(steps):
            observation, reward, terminated, truncated, info = self.step(action)
            if terminated or truncated:
                break
        moved = math.hypot(self.shark.x - start_x, self.shark.y - start_y)
        if moved < 0.02:
            raise RuntimeError(
                f"Gazebo motion smoke test failed: shark odometry did not change enough on {self.command_topic}. "
                f"Moved only {moved:.4f} m. Check the command topic path and robot command subscriber."
            )
