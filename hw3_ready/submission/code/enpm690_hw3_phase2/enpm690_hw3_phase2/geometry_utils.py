from __future__ import annotations

import math
import random

from .constants import Obstacle, WorldBounds


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_diff(target: float, source: float) -> float:
    return wrap_angle(target - source)


def distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def bearing_xy(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.atan2(y2 - y1, x2 - x1)


def point_in_bounds(x: float, y: float, bounds: WorldBounds, margin: float = 0.0) -> bool:
    return (
        bounds.x_min + margin <= x <= bounds.x_max - margin
        and bounds.y_min + margin <= y <= bounds.y_max - margin
    )


def nearest_point_on_rect(x: float, y: float, obstacle: Obstacle) -> tuple[float, float]:
    return (
        clamp(x, obstacle.xmin, obstacle.xmax),
        clamp(y, obstacle.ymin, obstacle.ymax),
    )


def circle_intersects_rect(x: float, y: float, radius: float, obstacle: Obstacle) -> bool:
    nearest_x, nearest_y = nearest_point_on_rect(x, y, obstacle)
    return distance_xy(x, y, nearest_x, nearest_y) <= radius


def push_circle_out_of_rect(x: float, y: float, radius: float, obstacle: Obstacle) -> tuple[float, float]:
    nearest_x, nearest_y = nearest_point_on_rect(x, y, obstacle)
    dx = x - nearest_x
    dy = y - nearest_y
    dist = math.hypot(dx, dy)

    if dist < 1e-6:
        left_pen = abs(x - obstacle.xmin)
        right_pen = abs(obstacle.xmax - x)
        bottom_pen = abs(y - obstacle.ymin)
        top_pen = abs(obstacle.ymax - y)
        smallest = min(left_pen, right_pen, bottom_pen, top_pen)
        if smallest == left_pen:
            return obstacle.xmin - radius, y
        if smallest == right_pen:
            return obstacle.xmax + radius, y
        if smallest == bottom_pen:
            return x, obstacle.ymin - radius
        return x, obstacle.ymax + radius

    scale = (radius - dist) / dist + 1.0
    return nearest_x + dx * scale, nearest_y + dy * scale


def reflect_heading_from_rect(x: float, y: float, heading: float, obstacle: Obstacle) -> float:
    nearest_x, nearest_y = nearest_point_on_rect(x, y, obstacle)
    dx = x - nearest_x
    dy = y - nearest_y
    if abs(dx) >= abs(dy):
        return wrap_angle(math.pi - heading)
    return wrap_angle(-heading)


def clamp_to_world(x: float, y: float, radius: float, bounds: WorldBounds) -> tuple[float, float]:
    return (
        clamp(x, bounds.x_min + radius, bounds.x_max - radius),
        clamp(y, bounds.y_min + radius, bounds.y_max - radius),
    )


def reflect_heading_from_bounds(x: float, y: float, heading: float, radius: float, bounds: WorldBounds) -> float:
    reflected = heading
    if x <= bounds.x_min + radius or x >= bounds.x_max - radius:
        reflected = wrap_angle(math.pi - reflected)
    if y <= bounds.y_min + radius or y >= bounds.y_max - radius:
        reflected = wrap_angle(-reflected)
    return reflected


def is_point_free(
    x: float,
    y: float,
    radius: float,
    bounds: WorldBounds,
    obstacles: list[Obstacle] | tuple[Obstacle, ...],
    occupied: list[tuple[float, float, float]] | None = None,
) -> bool:
    if not point_in_bounds(x, y, bounds, margin=radius):
        return False
    for obstacle in obstacles:
        if circle_intersects_rect(x, y, radius, obstacle):
            return False
    for occ_x, occ_y, occ_radius in occupied or []:
        if distance_xy(x, y, occ_x, occ_y) < radius + occ_radius:
            return False
    return True


def sample_free_point(
    rng: random.Random,
    bounds: WorldBounds,
    obstacles: list[Obstacle] | tuple[Obstacle, ...],
    radius: float,
    occupied: list[tuple[float, float, float]] | None = None,
    x_range: tuple[float, float] | None = None,
    y_range: tuple[float, float] | None = None,
    max_attempts: int = 500,
) -> tuple[float, float]:
    min_x = x_range[0] if x_range else bounds.x_min + radius
    max_x = x_range[1] if x_range else bounds.x_max - radius
    min_y = y_range[0] if y_range else bounds.y_min + radius
    max_y = y_range[1] if y_range else bounds.y_max - radius

    for _ in range(max_attempts):
        x = rng.uniform(min_x, max_x)
        y = rng.uniform(min_y, max_y)
        if is_point_free(x, y, radius, bounds, obstacles, occupied):
            return x, y
    return clamp_to_world(0.0, 0.0, radius, bounds)


def sample_near_obstacle(
    rng: random.Random,
    bounds: WorldBounds,
    obstacles: list[Obstacle] | tuple[Obstacle, ...],
    radius: float,
    occupied: list[tuple[float, float, float]] | None = None,
    band: tuple[float, float] = (0.20, 0.45),
    max_attempts: int = 500,
) -> tuple[float, float]:
    expanded = list(obstacles)
    wall_obstacles = [
        Obstacle("north_wall_band", 0.0, bounds.y_max + band[0] / 2.0, bounds.width, band[1]),
        Obstacle("south_wall_band", 0.0, bounds.y_min - band[0] / 2.0, bounds.width, band[1]),
        Obstacle("east_wall_band", bounds.x_max + band[0] / 2.0, 0.0, band[1], bounds.height),
        Obstacle("west_wall_band", bounds.x_min - band[0] / 2.0, 0.0, band[1], bounds.height),
    ]
    expanded.extend(wall_obstacles)

    for _ in range(max_attempts):
        obstacle = rng.choice(expanded)
        side = rng.choice(["left", "right", "top", "bottom"])
        offset = rng.uniform(band[0], band[1])
        if side == "left":
            x = obstacle.xmin - offset
            y = rng.uniform(obstacle.ymin, obstacle.ymax)
        elif side == "right":
            x = obstacle.xmax + offset
            y = rng.uniform(obstacle.ymin, obstacle.ymax)
        elif side == "top":
            x = rng.uniform(obstacle.xmin, obstacle.xmax)
            y = obstacle.ymax + offset
        else:
            x = rng.uniform(obstacle.xmin, obstacle.xmax)
            y = obstacle.ymin - offset

        if is_point_free(x, y, radius, bounds, obstacles, occupied):
            return x, y
    return sample_free_point(rng, bounds, obstacles, radius, occupied)


def raycast_world(
    origin_x: float,
    origin_y: float,
    angle: float,
    range_max: float,
    bounds: WorldBounds,
    obstacles: list[Obstacle] | tuple[Obstacle, ...],
) -> float:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    candidates: list[float] = []

    if abs(cos_a) > 1e-9:
        tx1 = (bounds.x_min - origin_x) / cos_a
        tx2 = (bounds.x_max - origin_x) / cos_a
        for tx in (tx1, tx2):
            if tx > 0:
                y = origin_y + tx * sin_a
                if bounds.y_min <= y <= bounds.y_max:
                    candidates.append(tx)

    if abs(sin_a) > 1e-9:
        ty1 = (bounds.y_min - origin_y) / sin_a
        ty2 = (bounds.y_max - origin_y) / sin_a
        for ty in (ty1, ty2):
            if ty > 0:
                x = origin_x + ty * cos_a
                if bounds.x_min <= x <= bounds.x_max:
                    candidates.append(ty)

    for obstacle in obstacles:
        tmin = 0.0
        tmax = range_max
        if abs(cos_a) < 1e-9:
            if not obstacle.xmin <= origin_x <= obstacle.xmax:
                continue
        else:
            tx1 = (obstacle.xmin - origin_x) / cos_a
            tx2 = (obstacle.xmax - origin_x) / cos_a
            tmin = max(tmin, min(tx1, tx2))
            tmax = min(tmax, max(tx1, tx2))
        if abs(sin_a) < 1e-9:
            if not obstacle.ymin <= origin_y <= obstacle.ymax:
                continue
        else:
            ty1 = (obstacle.ymin - origin_y) / sin_a
            ty2 = (obstacle.ymax - origin_y) / sin_a
            tmin = max(tmin, min(ty1, ty2))
            tmax = min(tmax, max(ty1, ty2))
        if tmax >= max(tmin, 0.0):
            candidates.append(max(tmin, 0.0))

    positive = [distance for distance in candidates if distance >= 0.0]
    if not positive:
        return range_max
    return min(range_max, min(positive))
