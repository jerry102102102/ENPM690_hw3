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

SHARK_MAX_LINEAR_SPEED = 0.6
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
PHASE2_OBSTACLES: tuple[Obstacle, ...] = ()
PACMAN_OBJECT_LAYOUT: tuple[tuple[str, str, float, float], ...] = (
    ("tuna_0", "tuna", -3.6, 3.4),
    ("tuna_1", "tuna", 0.0, 3.6),
    ("tuna_2", "tuna", 3.4, 3.0),
    ("tuna_3", "tuna", -3.0, -3.3),
    ("tuna_4", "tuna", 3.1, -3.4),
    ("sardine_0", "sardine", -2.4, 2.2),
    ("sardine_1", "sardine", -1.2, 2.0),
    ("sardine_2", "sardine", 1.2, 2.0),
    ("sardine_3", "sardine", 2.4, 2.2),
    ("sardine_4", "sardine", -2.2, 0.9),
    ("sardine_5", "sardine", 2.2, 0.9),
    ("sardine_6", "sardine", -2.2, -0.9),
    ("sardine_7", "sardine", 2.2, -0.9),
    ("sardine_8", "sardine", -1.2, -2.0),
    ("sardine_9", "sardine", 1.2, -2.0),
    ("seaweed_0", "seaweed", -3.7, 0.0),
    ("seaweed_1", "seaweed", -2.8, 0.0),
    ("seaweed_2", "seaweed", -1.8, 0.0),
    ("seaweed_3", "seaweed", -0.9, 0.0),
    ("seaweed_4", "seaweed", 0.9, 0.0),
    ("seaweed_5", "seaweed", 1.8, 0.0),
    ("seaweed_6", "seaweed", 2.8, 0.0),
    ("seaweed_7", "seaweed", 3.7, 0.0),
)

DEFAULT_SHARK_SPAWN = (0.0, 0.0, 0.0)
PACMAN_WORLD_BOUNDS = WorldBounds(x_min=-6.7, x_max=6.7, y_min=-6.7, y_max=6.7)
PACMAN_PELLET_RADIUS = 0.12
PACMAN_PELLET_SCORE = 1
PACMAN_GHOST_RADIUS = 0.22
PACMAN_GHOST_SPEED = 0.35
PACMAN_TIME_LIMIT = 30.0
PACMAN_PELLET_LAYOUT: tuple[tuple[str, float, float], ...] = (
    ("pellet_0", -5.4, 0.0),
    ("pellet_1", -4.2, 0.0),
    ("pellet_2", -3.0, 0.0),
    ("pellet_3", -1.8, 0.0),
    ("pellet_4", -0.6, 0.0),
    ("pellet_5", 0.6, 0.0),
    ("pellet_6", 3.2, 0.0),
    ("pellet_7", 4.6, 0.0),
    ("pellet_8", -4.8, 4.8),
    ("pellet_9", -2.0, 4.8),
    ("pellet_10", 0.8, 4.8),
    ("pellet_11", 4.6, 4.8),
    ("pellet_12", -4.8, -4.8),
    ("pellet_13", -2.0, -4.8),
    ("pellet_14", 0.8, -4.8),
    ("pellet_15", 4.6, -4.8),
)
PACMAN_GHOST_WAYPOINTS: tuple[tuple[float, float], ...] = (
    (-4.6, 2.0),
    (-1.0, 2.0),
    (-1.0, -2.8),
    (-4.6, -2.8),
)


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
class PelletState:
    pellet_id: str
    x: float
    y: float
    active: bool = True
    value: int = PACMAN_PELLET_SCORE
    radius: float = PACMAN_PELLET_RADIUS

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class GhostState:
    ghost_id: str
    x: float
    y: float
    heading: float
    speed: float
    radius: float
    waypoint_index: int = 0
    active: bool = True

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class PacmanGameSnapshot:
    mode: str
    score: int
    time_remaining: float
    pellets_remaining: int
    game_over: bool = False
    victory: bool = False
    sync_ready: bool = False
    collision_cooldown: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class GameSnapshot:
    mode: str
    score: int
    time_remaining: float
    shark: SharkState
    sync_ready: bool = False
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
            "sync_ready": self.sync_ready,
            "catch_counts": dict(self.catch_counts),
            "collision_cooldown": self.collision_cooldown,
            "current_target_id": self.current_target_id,
            "current_target_species": self.current_target_species,
        }
