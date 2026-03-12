from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from .constants import (
    LIDAR_BIN_COUNT,
    PHASE2_WORLD_BOUNDS,
    SHARK_MAX_ANGULAR_SPEED,
    SHARK_MAX_LINEAR_SPEED,
    GAME_DURATION_SECONDS,
    SHARK_STUN_SECONDS,
    SharkState,
)
from .geometry_utils import angle_diff, bearing_xy, clamp, distance_xy


class ObservationBuilder:
    @staticmethod
    def preprocess_scan_ranges(
        ranges: Iterable[float],
        range_min: float,
        range_max: float,
        bin_count: int = LIDAR_BIN_COUNT,
    ) -> np.ndarray:
        values = np.asarray(list(ranges), dtype=np.float32)
        if values.size == 0:
            return np.zeros(bin_count, dtype=np.float32)
        values = np.nan_to_num(values, nan=range_max, posinf=range_max, neginf=range_max)
        values = np.clip(values, range_min, range_max)
        split = np.array_split(values, bin_count)
        pooled = np.array([chunk.min() if len(chunk) > 0 else range_max for chunk in split], dtype=np.float32)
        proximity = 1.0 - pooled / max(range_max, 1e-6)
        return np.clip(proximity, 0.0, 1.0)

    @staticmethod
    def front_sector_proximities(binned_proximity: np.ndarray) -> tuple[float, float, float]:
        front_center = len(binned_proximity) // 2
        left = float(np.max(binned_proximity[front_center + 1 : front_center + 5]))
        center = float(np.max(binned_proximity[front_center - 1 : front_center + 2]))
        right = float(np.max(binned_proximity[front_center - 5 : front_center - 1]))
        return left, center, right

    @staticmethod
    def nearest_species_features(shark: SharkState, fish_states: list, species: str) -> tuple[float, float, float]:
        active = [fish for fish in fish_states if fish.active and fish.species == species]
        if not active:
            return 1.0, 0.0, 1.0
        target = min(active, key=lambda fish: distance_xy(shark.x, shark.y, fish.x, fish.y))
        distance = distance_xy(shark.x, shark.y, target.x, target.y)
        bearing = angle_diff(bearing_xy(shark.x, shark.y, target.x, target.y), shark.heading)
        normalized_distance = clamp(distance / max(PHASE2_WORLD_BOUNDS.width, PHASE2_WORLD_BOUNDS.height), 0.0, 1.0)
        return normalized_distance, math.sin(bearing), math.cos(bearing)

    @staticmethod
    def build_vector(
        shark: SharkState,
        time_remaining: float,
        lidar_proximity: np.ndarray,
        fish_states: list,
    ) -> np.ndarray:
        x_norm = (shark.x - PHASE2_WORLD_BOUNDS.x_min) / PHASE2_WORLD_BOUNDS.width
        y_norm = (shark.y - PHASE2_WORLD_BOUNDS.y_min) / PHASE2_WORLD_BOUNDS.height
        tuna = ObservationBuilder.nearest_species_features(shark, fish_states, "tuna")
        sardine = ObservationBuilder.nearest_species_features(shark, fish_states, "sardine")
        seaweed = ObservationBuilder.nearest_species_features(shark, fish_states, "seaweed")
        vector = np.array(
            [
                clamp(x_norm, 0.0, 1.0),
                clamp(y_norm, 0.0, 1.0),
                math.cos(shark.heading),
                math.sin(shark.heading),
                clamp(shark.linear_speed / SHARK_MAX_LINEAR_SPEED, 0.0, 1.0),
                clamp((shark.angular_speed / SHARK_MAX_ANGULAR_SPEED + 1.0) * 0.5, 0.0, 1.0),
                clamp(shark.collision_cooldown / SHARK_STUN_SECONDS, 0.0, 1.0),
                clamp(time_remaining / GAME_DURATION_SECONDS, 0.0, 1.0),
            ]
            + lidar_proximity.astype(np.float32).tolist()
            + list(tuna)
            + list(sardine)
            + list(seaweed),
            dtype=np.float32,
        )
        return vector
