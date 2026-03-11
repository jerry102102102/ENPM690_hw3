from __future__ import annotations

import math
import random

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .constants import (
    GAME_DT,
    GAME_DURATION_SECONDS,
    LIDAR_BIN_COUNT,
    PHASE2_OBSTACLES,
    PHASE2_WORLD_BOUNDS,
    SHARK_MAX_ANGULAR_SPEED,
    SHARK_MAX_LINEAR_SPEED,
    SHARK_RADIUS,
    SHARK_STUN_SECONDS,
    DEFAULT_SHARK_SPAWN,
    SharkState,
)
from .fish_manager import FishManager
from .geometry_utils import raycast_world, wrap_angle, distance_xy, bearing_xy, angle_diff
from .observation_builder import ObservationBuilder
from .reward_builder import RewardBuilder
from .shark_collision_monitor import SharkCollisionMonitor


class SharkHuntLogicalEnv(gym.Env[np.ndarray, np.ndarray]):
    metadata = {"render_modes": []}

    def __init__(self) -> None:
        super().__init__()
        self.rng = random.Random(690)
        self.fish_manager = FishManager(PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES, self.rng)
        self.collision_monitor = SharkCollisionMonitor(PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES, SHARK_RADIUS)
        self.reward_builder = RewardBuilder()

        self.dt = GAME_DT
        self.max_steps = int(GAME_DURATION_SECONDS / self.dt)
        self.range_min = 0.12
        self.range_max = 6.0

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

        self.shark = SharkState()
        self.steps = 0
        self.score = 0
        self.catch_counts = {"tuna": 0, "sardine": 0, "seaweed": 0}

    def reset(self, *, seed: int | None = None, options: dict | None = None) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        if seed is not None:
            self.rng.seed(seed)
        self.shark = SharkState(x=DEFAULT_SHARK_SPAWN[0], y=DEFAULT_SHARK_SPAWN[1], heading=DEFAULT_SHARK_SPAWN[2])
        self.steps = 0
        self.score = 0
        self.catch_counts = {"tuna": 0, "sardine": 0, "seaweed": 0}
        self.fish_manager.reset()
        observation = self._build_observation()
        return observation, self._build_info(collision=False, reward_components={})

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)

        target_before = self._current_target()
        distance_before = self._distance_to_target(target_before)
        collision = False

        if self.shark.collision_cooldown > 0.0:
            linear_cmd = 0.0
            angular_cmd = 0.0
            self.shark.collision_cooldown = max(0.0, self.shark.collision_cooldown - self.dt)
        else:
            linear_cmd = 0.5 * (float(action[0]) + 1.0) * SHARK_MAX_LINEAR_SPEED
            angular_cmd = float(action[1]) * SHARK_MAX_ANGULAR_SPEED

        self.shark.angular_speed = angular_cmd
        self.shark.linear_speed = linear_cmd
        self.shark.heading = wrap_angle(self.shark.heading + angular_cmd * self.dt)
        prev_x, prev_y = self.shark.x, self.shark.y
        self.shark.x += math.cos(self.shark.heading) * linear_cmd * self.dt
        self.shark.y += math.sin(self.shark.heading) * linear_cmd * self.dt

        if self.collision_monitor.check_collision(self.shark):
            collision = True
            self.shark.x = prev_x
            self.shark.y = prev_y
            self.shark.linear_speed = 0.0
            self.shark.angular_speed = 0.0
            self.shark.collision_cooldown = SHARK_STUN_SECONDS

        self.fish_manager.update(self.dt, self.shark, immediate_respawn=True)
        catches = self.fish_manager.detect_catches(self.shark, immediate_respawn=True)
        catch_score = sum(event.score_delta for event in catches)
        for event in catches:
            self.score += event.score_delta
            self.catch_counts[event.species] += 1

        self.steps += 1
        distance_after = self._distance_to_target(target_before)
        lidar = self._logical_lidar()
        _, front_prox, _ = ObservationBuilder.front_sector_proximities(lidar)
        reward, components = self.reward_builder.compute(
            catch_score=catch_score,
            collision=collision,
            distance_before=distance_before,
            distance_after=distance_after,
            front_obstacle_proximity=front_prox,
            speed_fraction=self.shark.linear_speed / max(SHARK_MAX_LINEAR_SPEED, 1e-6),
        )

        observation = self._build_observation(lidar)
        terminated = False
        truncated = self.steps >= self.max_steps
        info = self._build_info(collision=collision, reward_components=components)
        info["score"] = self.score
        info["catch_counts"] = dict(self.catch_counts)
        return observation, float(reward), terminated, truncated, info

    def _logical_lidar(self) -> np.ndarray:
        raw = []
        for index in range(LIDAR_BIN_COUNT):
            angle = self.shark.heading - math.pi + (2.0 * math.pi * index / LIDAR_BIN_COUNT)
            raw.append(raycast_world(self.shark.x, self.shark.y, angle, self.range_max, PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES))
        return ObservationBuilder.preprocess_scan_ranges(raw, self.range_min, self.range_max, LIDAR_BIN_COUNT)

    def _current_target(self):
        best = None
        best_utility = -1.0
        for species, score in (("tuna", 10.0), ("sardine", 3.0), ("seaweed", 1.0)):
            candidate = self.fish_manager.nearest_active_by_species(species, self.shark)
            if candidate is None:
                continue
            distance = distance_xy(self.shark.x, self.shark.y, candidate.x, candidate.y)
            utility = score / (distance + 1e-3)
            if utility > best_utility:
                best = candidate
                best_utility = utility
        return best

    def _distance_to_target(self, target) -> float | None:
        if target is None:
            return None
        return distance_xy(self.shark.x, self.shark.y, target.x, target.y)

    def _build_observation(self, lidar: np.ndarray | None = None) -> np.ndarray:
        if lidar is None:
            lidar = self._logical_lidar()
        time_remaining = max(0.0, GAME_DURATION_SECONDS - self.steps * self.dt)
        return ObservationBuilder.build_vector(self.shark, time_remaining, lidar, self.fish_manager.fish)

    def _build_info(self, collision: bool, reward_components: dict) -> dict:
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
            "time_remaining": max(0.0, GAME_DURATION_SECONDS - self.steps * self.dt),
        }
