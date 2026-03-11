from __future__ import annotations

from .constants import Obstacle, SharkState, WorldBounds
from .geometry_utils import circle_intersects_rect, clamp_to_world


class SharkCollisionMonitor:
    def __init__(self, bounds: WorldBounds, obstacles: tuple[Obstacle, ...], shark_radius: float) -> None:
        self.bounds = bounds
        self.obstacles = obstacles
        self.shark_radius = shark_radius

    def check_collision(self, shark: SharkState) -> bool:
        return any(circle_intersects_rect(shark.x, shark.y, self.shark_radius, obstacle) for obstacle in self.obstacles)

    def clamp_position(self, shark: SharkState) -> None:
        shark.x, shark.y = clamp_to_world(shark.x, shark.y, self.shark_radius, self.bounds)
