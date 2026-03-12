from __future__ import annotations

from .constants import PACMAN_PELLET_LAYOUT, PACMAN_PELLET_RADIUS, PACMAN_PELLET_SCORE, PelletState
from .geometry_utils import distance_xy


class PelletManager:
    def __init__(self) -> None:
        self.pellets: list[PelletState] = []

    def reset(self) -> list[PelletState]:
        self.pellets = [
            PelletState(
                pellet_id=pellet_id,
                x=x,
                y=y,
                active=True,
                value=PACMAN_PELLET_SCORE,
                radius=PACMAN_PELLET_RADIUS,
            )
            for pellet_id, x, y in PACMAN_PELLET_LAYOUT
        ]
        return self.pellets

    def collect_near(self, x: float, y: float, catch_radius: float) -> list[PelletState]:
        collected: list[PelletState] = []
        for pellet in self.pellets:
            if not pellet.active:
                continue
            if distance_xy(x, y, pellet.x, pellet.y) <= catch_radius + pellet.radius:
                pellet.active = False
                collected.append(pellet)
        return collected

    def active_pellets(self) -> list[PelletState]:
        return [pellet for pellet in self.pellets if pellet.active]

    def active_count(self) -> int:
        return sum(1 for pellet in self.pellets if pellet.active)
