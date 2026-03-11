from __future__ import annotations

import math
import random

from .constants import FishState, SharkState, SPECIES_CONFIGS
from .geometry_utils import angle_diff, bearing_xy, clamp, wrap_angle


def tuna_heading_update(fish: FishState, shark: SharkState | None, rng: random.Random, dt: float) -> float:
    desired = fish.heading + rng.uniform(-0.18, 0.18) * dt
    if shark is not None:
        shark_distance = math.hypot(shark.x - fish.x, shark.y - fish.y)
        if shark_distance < 0.9:
            away = bearing_xy(shark.x, shark.y, fish.x, fish.y)
            desired = wrap_angle(0.7 * desired + 0.3 * away)
    delta = clamp(angle_diff(desired, fish.heading), -SPECIES_CONFIGS["tuna"].max_turn_rate * dt, SPECIES_CONFIGS["tuna"].max_turn_rate * dt)
    return wrap_angle(fish.heading + delta)


def sardine_heading_update(
    fish: FishState,
    neighbors: list[FishState],
    shark: SharkState | None,
    rng: random.Random,
    dt: float,
) -> float:
    if not neighbors:
        noise = rng.uniform(-0.5, 0.5) * dt
        return wrap_angle(fish.heading + noise)

    sep_x = 0.0
    sep_y = 0.0
    coh_x = 0.0
    coh_y = 0.0
    ali_x = 0.0
    ali_y = 0.0

    for neighbor in neighbors:
        dx = fish.x - neighbor.x
        dy = fish.y - neighbor.y
        dist = math.hypot(dx, dy) + 1e-6
        sep_x += dx / dist**2
        sep_y += dy / dist**2
        coh_x += neighbor.x
        coh_y += neighbor.y
        ali_x += math.cos(neighbor.heading)
        ali_y += math.sin(neighbor.heading)

    coh_x = coh_x / len(neighbors) - fish.x
    coh_y = coh_y / len(neighbors) - fish.y

    flee_x = 0.0
    flee_y = 0.0
    if shark is not None:
        shark_distance = math.hypot(shark.x - fish.x, shark.y - fish.y)
        if shark_distance < 1.6:
            flee_x = fish.x - shark.x
            flee_y = fish.y - shark.y

    steer_x = 1.4 * sep_x + 0.6 * coh_x + 0.8 * ali_x + 2.0 * flee_x
    steer_y = 1.4 * sep_y + 0.6 * coh_y + 0.8 * ali_y + 2.0 * flee_y

    if abs(steer_x) < 1e-6 and abs(steer_y) < 1e-6:
        desired = fish.heading
    else:
        desired = math.atan2(steer_y, steer_x)

    desired = wrap_angle(desired + rng.uniform(-0.12, 0.12) * dt)
    max_turn = SPECIES_CONFIGS["sardine"].max_turn_rate * dt
    delta = clamp(angle_diff(desired, fish.heading), -max_turn, max_turn)
    return wrap_angle(fish.heading + delta)
