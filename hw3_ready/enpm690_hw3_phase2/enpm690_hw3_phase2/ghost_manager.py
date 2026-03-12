from __future__ import annotations

import math

from .constants import PACMAN_GHOST_RADIUS, PACMAN_GHOST_SPEED, PACMAN_GHOST_WAYPOINTS, GhostState
from .geometry_utils import distance_xy, wrap_angle


class GhostManager:
    def __init__(self) -> None:
        self.ghost = GhostState(
            ghost_id="ghost_0",
            x=PACMAN_GHOST_WAYPOINTS[0][0],
            y=PACMAN_GHOST_WAYPOINTS[0][1],
            heading=0.0,
            speed=PACMAN_GHOST_SPEED,
            radius=PACMAN_GHOST_RADIUS,
            waypoint_index=1,
            active=True,
        )

    def reset(self) -> GhostState:
        self.ghost = GhostState(
            ghost_id="ghost_0",
            x=PACMAN_GHOST_WAYPOINTS[0][0],
            y=PACMAN_GHOST_WAYPOINTS[0][1],
            heading=0.0,
            speed=PACMAN_GHOST_SPEED,
            radius=PACMAN_GHOST_RADIUS,
            waypoint_index=1,
            active=True,
        )
        return self.ghost

    def update(self, dt: float) -> GhostState:
        waypoint_x, waypoint_y = PACMAN_GHOST_WAYPOINTS[self.ghost.waypoint_index]
        dx = waypoint_x - self.ghost.x
        dy = waypoint_y - self.ghost.y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            self.ghost.waypoint_index = (self.ghost.waypoint_index + 1) % len(PACMAN_GHOST_WAYPOINTS)
            return self.update(dt)

        self.ghost.heading = wrap_angle(math.atan2(dy, dx))
        step = min(self.ghost.speed * dt, distance)
        self.ghost.x += math.cos(self.ghost.heading) * step
        self.ghost.y += math.sin(self.ghost.heading) * step
        if distance_xy(self.ghost.x, self.ghost.y, waypoint_x, waypoint_y) < 0.12:
            self.ghost.waypoint_index = (self.ghost.waypoint_index + 1) % len(PACMAN_GHOST_WAYPOINTS)
        return self.ghost
