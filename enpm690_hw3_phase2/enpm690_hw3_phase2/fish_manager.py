from __future__ import annotations

import random

from .constants import (
    CatchEvent,
    FishState,
    PACMAN_OBJECT_LAYOUT,
    SHARK_CATCH_RADIUS,
    SPECIES_CONFIGS,
    SharkState,
    WorldBounds,
    Obstacle,
)
from .geometry_utils import distance_xy


class FishManager:
    def __init__(
        self,
        bounds: WorldBounds,
        obstacles: tuple[Obstacle, ...],
        rng: random.Random | None = None,
        species_speed_scales: dict[str, float] | None = None,
    ) -> None:
        self.bounds = bounds
        self.obstacles = obstacles
        self.rng = rng or random.Random()
        self.species_speed_scales = species_speed_scales or {}
        self.fish: list[FishState] = []
        self.spawn_lookup = {
            fish_id: (species, x, y) for fish_id, species, x, y in PACMAN_OBJECT_LAYOUT
        }

    def reset(self) -> list[FishState]:
        self.fish = []
        for fish_id, species, x, y in PACMAN_OBJECT_LAYOUT:
            self.fish.append(
                FishState(
                    fish_id=fish_id,
                    species=species,
                    x=x,
                    y=y,
                    heading=0.0,
                    speed=0.0,
                    radius=SPECIES_CONFIGS[species].radius,
                )
            )
        return self.fish

    def update(
        self,
        dt: float,
        shark: SharkState | None = None,
        immediate_respawn: bool = False,
        respawn_enabled: bool = True,
    ) -> None:
        for fish in self.fish:
            if not fish.active:
                if not respawn_enabled:
                    continue
                fish.respawn_timer -= dt
                if fish.respawn_timer <= 0.0:
                    self._respawn_single(fish, immediate=immediate_respawn)
                continue

    def detect_catches(
        self,
        shark: SharkState,
        immediate_respawn: bool = False,
        respawn_enabled: bool = True,
    ) -> list[CatchEvent]:
        catches: list[CatchEvent] = []
        for fish in self.fish:
            if not fish.active:
                continue
            if distance_xy(shark.x, shark.y, fish.x, fish.y) <= SHARK_CATCH_RADIUS + fish.radius:
                fish.active = False
                fish.respawn_timer = 0.0 if (immediate_respawn and respawn_enabled) else -1.0
                catches.append(
                    CatchEvent(
                        fish_id=fish.fish_id,
                        species=fish.species,
                        score_delta=SPECIES_CONFIGS[fish.species].score,
                    )
                )
                if immediate_respawn and respawn_enabled:
                    self._respawn_single(fish, immediate=True)
        return catches

    def nearest_active_by_species(self, species: str, shark: SharkState) -> FishState | None:
        active = [fish for fish in self.fish if fish.active and fish.species == species]
        if not active:
            return None
        return min(active, key=lambda fish: distance_xy(shark.x, shark.y, fish.x, fish.y))

    def _respawn_single(self, fish: FishState, immediate: bool = False) -> None:
        _, x, y = self.spawn_lookup[fish.fish_id]
        fish.x = x
        fish.y = y
        fish.heading = 0.0
        fish.speed = 0.0
        fish.active = True
        fish.respawn_timer = 0.0

    def active_count(self) -> int:
        return sum(1 for fish in self.fish if fish.active)
