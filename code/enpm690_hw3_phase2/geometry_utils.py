from __future__ import annotations

import math

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
