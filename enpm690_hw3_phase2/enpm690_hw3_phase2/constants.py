from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


MODE_TELEOP = "teleop_play"
MODE_AUTO = "auto_play"
MODE_TRAIN = "train_headless"
MODE_EVAL = "eval_demo"

GAME_DURATION_SECONDS = 30.0
GAME_UPDATE_HZ = 10.0
GAME_DT = 1.0 / GAME_UPDATE_HZ
LIDAR_BIN_COUNT = 24

SHARK_MAX_LINEAR_SPEED = 1.0
SHARK_MAX_ANGULAR_SPEED = 2.0
SHARK_RADIUS = 0.16
SHARK_CATCH_RADIUS = 0.18
SHARK_STUN_SECONDS = 1.0


@dataclass(frozen=True)
class SpeciesConfig:
    name: str
    count: int
    score: int
    speed_scale: float
    radius: float
    color_rgba: tuple[float, float, float, float]
    max_turn_rate: float


SPECIES_CONFIGS: dict[str, SpeciesConfig] = {
    "tuna": SpeciesConfig(
        name="tuna",
        count=5,
        score=10,
        speed_scale=0.8,
        radius=0.18,
        color_rgba=(0.10, 0.50, 0.75, 0.95),
        max_turn_rate=0.55,
    ),
    "sardine": SpeciesConfig(
        name="sardine",
        count=10,
        score=3,
        speed_scale=0.4,
        radius=0.13,
        color_rgba=(0.85, 0.85, 0.45, 0.95),
        max_turn_rate=1.10,
    ),
    "seaweed": SpeciesConfig(
        name="seaweed",
        count=8,
        score=1,
        speed_scale=0.0,
        radius=0.10,
        color_rgba=(0.20, 0.75, 0.25, 0.95),
        max_turn_rate=0.0,
    ),
}


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


PHASE2_WORLD_BOUNDS = WorldBounds(x_min=-4.8, x_max=4.8, y_min=-4.8, y_max=4.8)
PHASE2_OBSTACLES: tuple[Obstacle, ...] = (
    Obstacle("center_reef", 1.2, 0.0, 1.0, 1.0),
    Obstacle("north_reef", 0.0, 2.1, 1.2, 0.8),
    Obstacle("south_reef", -0.6, -2.1, 1.2, 0.8),
    Obstacle("east_column", 2.8, 1.8, 0.8, 1.4),
    Obstacle("west_column", -2.6, 1.0, 0.9, 1.6),
    Obstacle("lower_east_block", 2.5, -1.8, 1.0, 1.0),
)

DEFAULT_SHARK_SPAWN = (0.0, 0.0, 0.0)


@dataclass
class FishState:
    fish_id: str
    species: str
    x: float
    y: float
    heading: float
    speed: float
    radius: float
    active: bool = True
    respawn_timer: float = 0.0
    school_id: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SharkState:
    x: float = DEFAULT_SHARK_SPAWN[0]
    y: float = DEFAULT_SHARK_SPAWN[1]
    heading: float = DEFAULT_SHARK_SPAWN[2]
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    collision_cooldown: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class CatchEvent:
    fish_id: str
    species: str
    score_delta: int


@dataclass
class GameSnapshot:
    mode: str
    score: int
    time_remaining: float
    shark: SharkState
    catch_counts: dict[str, int] = field(default_factory=lambda: {"tuna": 0, "sardine": 0, "seaweed": 0})
    collision_cooldown: float = 0.0
    current_target_id: str = ""
    current_target_species: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "score": self.score,
            "time_remaining": self.time_remaining,
            "shark": self.shark.to_dict(),
            "catch_counts": dict(self.catch_counts),
            "collision_cooldown": self.collision_cooldown,
            "current_target_id": self.current_target_id,
            "current_target_species": self.current_target_species,
        }
