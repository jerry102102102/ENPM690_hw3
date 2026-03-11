from __future__ import annotations

import math
import random

from .constants import (
    CatchEvent,
    FishState,
    SHARK_CATCH_RADIUS,
    SHARK_MAX_LINEAR_SPEED,
    SPECIES_CONFIGS,
    SharkState,
    WorldBounds,
    Obstacle,
)
from .geometry_utils import (
    distance_xy,
    reflect_heading_from_bounds,
    reflect_heading_from_rect,
    sample_free_point,
    sample_near_obstacle,
    clamp_to_world,
    circle_intersects_rect,
    push_circle_out_of_rect,
)
from .prey_behaviors import sardine_heading_update, tuna_heading_update


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

    def reset(self) -> list[FishState]:
        self.fish = []
        occupied: list[tuple[float, float, float]] = []

        for index in range(SPECIES_CONFIGS["tuna"].count):
            x, y = sample_free_point(
                self.rng,
                self.bounds,
                self.obstacles,
                SPECIES_CONFIGS["tuna"].radius,
                occupied,
                x_range=(-3.4, 3.8),
                y_range=(-3.8, 3.8),
            )
            fish = FishState(
                fish_id=f"tuna_{index}",
                species="tuna",
                x=x,
                y=y,
                heading=self.rng.uniform(-math.pi, math.pi),
                speed=self._speed_for_species("tuna"),
                radius=SPECIES_CONFIGS["tuna"].radius,
            )
            self.fish.append(fish)
            occupied.append((x, y, fish.radius))

        school_centers = [(-2.0, -0.6), (2.4, 1.4)]
        sardines_per_school = SPECIES_CONFIGS["sardine"].count // len(school_centers)
        for school_id, (center_x, center_y) in enumerate(school_centers):
            for offset in range(sardines_per_school):
                x, y = sample_free_point(
                    self.rng,
                    self.bounds,
                    self.obstacles,
                    SPECIES_CONFIGS["sardine"].radius,
                    occupied,
                    x_range=(center_x - 0.8, center_x + 0.8),
                    y_range=(center_y - 0.8, center_y + 0.8),
                )
                fish = FishState(
                    fish_id=f"sardine_{school_id * sardines_per_school + offset}",
                    species="sardine",
                    x=x,
                    y=y,
                    heading=self.rng.uniform(-math.pi, math.pi),
                    speed=self._speed_for_species("sardine"),
                    radius=SPECIES_CONFIGS["sardine"].radius,
                    school_id=school_id,
                )
                self.fish.append(fish)
                occupied.append((x, y, fish.radius))

        for index in range(SPECIES_CONFIGS["seaweed"].count):
            x, y = sample_near_obstacle(
                self.rng,
                self.bounds,
                self.obstacles,
                SPECIES_CONFIGS["seaweed"].radius,
                occupied,
            )
            fish = FishState(
                fish_id=f"seaweed_{index}",
                species="seaweed",
                x=x,
                y=y,
                heading=0.0,
                speed=0.0,
                radius=SPECIES_CONFIGS["seaweed"].radius,
            )
            self.fish.append(fish)
            occupied.append((x, y, fish.radius))

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

            if fish.species == "seaweed":
                continue

            if fish.species == "tuna":
                fish.heading = tuna_heading_update(fish, shark, self.rng, dt)
            elif fish.species == "sardine":
                schoolmates = [
                    candidate
                    for candidate in self.fish
                    if candidate.active
                    and candidate.species == "sardine"
                    and candidate.school_id == fish.school_id
                    and candidate.fish_id != fish.fish_id
                    and distance_xy(candidate.x, candidate.y, fish.x, fish.y) < 1.6
                ]
                fish.heading = sardine_heading_update(fish, schoolmates, shark, self.rng, dt)

            fish.x += math.cos(fish.heading) * fish.speed * dt
            fish.y += math.sin(fish.heading) * fish.speed * dt
            fish.x, fish.y = clamp_to_world(fish.x, fish.y, fish.radius, self.bounds)
            fish.heading = reflect_heading_from_bounds(fish.x, fish.y, fish.heading, fish.radius, self.bounds)

            for obstacle in self.obstacles:
                if circle_intersects_rect(fish.x, fish.y, fish.radius, obstacle):
                    fish.x, fish.y = push_circle_out_of_rect(fish.x, fish.y, fish.radius, obstacle)
                    fish.heading = reflect_heading_from_rect(fish.x, fish.y, fish.heading, obstacle)

        self._resolve_fish_collisions()

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
        occupied = [(candidate.x, candidate.y, candidate.radius) for candidate in self.fish if candidate.active and candidate.fish_id != fish.fish_id]
        if fish.species == "seaweed":
            x, y = sample_near_obstacle(self.rng, self.bounds, self.obstacles, fish.radius, occupied)
            fish.heading = 0.0
            fish.speed = 0.0
        elif fish.species == "tuna":
            x, y = sample_free_point(self.rng, self.bounds, self.obstacles, fish.radius, occupied, x_range=(-3.5, 3.8), y_range=(-3.8, 3.8))
            fish.heading = self.rng.uniform(-math.pi, math.pi)
            fish.speed = self._speed_for_species("tuna")
        else:
            school_center = (-2.0, -0.6) if fish.school_id == 0 else (2.4, 1.4)
            x, y = sample_free_point(
                self.rng,
                self.bounds,
                self.obstacles,
                fish.radius,
                occupied,
                x_range=(school_center[0] - 0.8, school_center[0] + 0.8),
                y_range=(school_center[1] - 0.8, school_center[1] + 0.8),
            )
            fish.heading = self.rng.uniform(-math.pi, math.pi)
            fish.speed = self._speed_for_species("sardine")

        fish.x = x
        fish.y = y
        fish.active = True
        fish.respawn_timer = 0.0

    def _resolve_fish_collisions(self) -> None:
        active = [fish for fish in self.fish if fish.active]
        for index, fish in enumerate(active):
            for other in active[index + 1 :]:
                dx = other.x - fish.x
                dy = other.y - fish.y
                dist = math.hypot(dx, dy)
                min_dist = fish.radius + other.radius
                if 0.0 < dist < min_dist:
                    overlap = min_dist - dist
                    nx = dx / dist
                    ny = dy / dist
                    fish.x -= 0.5 * overlap * nx
                    fish.y -= 0.5 * overlap * ny
                    other.x += 0.5 * overlap * nx
                    other.y += 0.5 * overlap * ny
                    fish.heading = math.atan2(-ny, -nx)
                    other.heading = math.atan2(ny, nx)

    def active_count(self) -> int:
        return sum(1 for fish in self.fish if fish.active)

    def _speed_for_species(self, species: str) -> float:
        scale = self.species_speed_scales.get(species, SPECIES_CONFIGS[species].speed_scale)
        return scale * SHARK_MAX_LINEAR_SPEED
