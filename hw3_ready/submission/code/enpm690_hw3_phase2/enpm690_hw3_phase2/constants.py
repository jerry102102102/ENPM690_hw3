from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WorldBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min


@dataclass(frozen=True)
class Obstacle:
    name: str
    x: float
    y: float
    width: float
    height: float

    @property
    def xmin(self) -> float:
        return self.x - self.width / 2.0

    @property
    def xmax(self) -> float:
        return self.x + self.width / 2.0

    @property
    def ymin(self) -> float:
        return self.y - self.height / 2.0

    @property
    def ymax(self) -> float:
        return self.y + self.height / 2.0
