from __future__ import annotations

from typing import Any

from .constants import (
    GAME_DT,
    GAME_DURATION_SECONDS,
    PHASE2_OBSTACLES,
    PHASE2_WORLD_BOUNDS,
    SHARK_RADIUS,
    SHARK_STUN_SECONDS,
    CatchEvent,
    GameSnapshot,
    SharkState,
)
from .fish_manager import FishManager
from .geometry_utils import angle_diff, bearing_xy, distance_xy
from .shark_collision_monitor import SharkCollisionMonitor


class GameCore:
    def __init__(
        self,
        *,
        episode_duration: float = GAME_DURATION_SECONDS,
        dt: float = GAME_DT,
        fish_manager: FishManager | None = None,
        collision_monitor: SharkCollisionMonitor | None = None,
    ) -> None:
        self.episode_duration = episode_duration
        self.dt = dt
        self.fish_manager = fish_manager or FishManager(PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES)
        self.collision_monitor = collision_monitor or SharkCollisionMonitor(PHASE2_WORLD_BOUNDS, PHASE2_OBSTACLES, SHARK_RADIUS)

        self.shark = SharkState()
        self.score = 0
        self.time_remaining = self.episode_duration
        self.catch_counts = {"tuna": 0, "sardine": 0, "seaweed": 0}
        self.current_target_id = ""
        self.current_target_species = ""

    def reset(self) -> None:
        self.fish_manager.reset()
        self.shark = SharkState()
        self.score = 0
        self.time_remaining = self.episode_duration
        self.catch_counts = {"tuna": 0, "sardine": 0, "seaweed": 0}
        self.current_target_id = ""
        self.current_target_species = ""
        self._update_target_snapshot()

    def consume_cooldown_tick(self) -> bool:
        if self.shark.collision_cooldown <= 0.0:
            return False
        self.shark.collision_cooldown = max(0.0, self.shark.collision_cooldown - self.dt)
        return True

    def trigger_collision_if_needed(self) -> bool:
        if self.shark.collision_cooldown > 0.0:
            return False
        if not self.collision_monitor.check_collision(self.shark):
            return False
        self.shark.collision_cooldown = SHARK_STUN_SECONDS
        return True

    def advance_episode(self, *, immediate_respawn: bool) -> list[CatchEvent]:
        self.fish_manager.update(self.dt, self.shark, immediate_respawn=immediate_respawn)
        catches = self.fish_manager.detect_catches(self.shark, immediate_respawn=immediate_respawn)
        for event in catches:
            self.score += event.score_delta
            self.catch_counts[event.species] += 1
        self.time_remaining = max(0.0, self.time_remaining - self.dt)
        self._update_target_snapshot()
        return catches

    def current_target(self):
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

    def distance_to_target(self, target) -> float | None:
        if target is None:
            return None
        return distance_xy(self.shark.x, self.shark.y, target.x, target.y)

    def build_info(self, *, collision: bool, reward_components: dict[str, float]) -> dict[str, Any]:
        target = self.current_target()
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

    def build_snapshot(self, *, mode: str, sync_ready: bool) -> GameSnapshot:
        return GameSnapshot(
            mode=mode,
            score=self.score,
            time_remaining=self.time_remaining,
            shark=self.shark,
            sync_ready=sync_ready,
            catch_counts=dict(self.catch_counts),
            collision_cooldown=self.shark.collision_cooldown,
            current_target_id=self.current_target_id,
            current_target_species=self.current_target_species,
        )

    def _update_target_snapshot(self) -> None:
        target = self.current_target()
        if target is None:
            self.current_target_id = ""
            self.current_target_species = ""
            return
        self.current_target_id = target.fish_id
        self.current_target_species = target.species
