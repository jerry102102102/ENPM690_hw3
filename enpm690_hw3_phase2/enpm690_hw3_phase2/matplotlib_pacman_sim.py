from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

from .constants import PACMAN_GHOST_WAYPOINTS, PACMAN_TIME_LIMIT, PACMAN_WORLD_BOUNDS, SHARK_RADIUS
from .geometry_utils import angle_diff, bearing_xy, clamp, clamp_to_world, distance_xy, raycast_world
from .ghost_manager import GhostManager
from .pellet_manager import PelletManager


@dataclass
class PacmanState:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    linear_speed: float = 0.0
    angular_speed: float = 0.0


class MatplotlibPacmanSim:
    def __init__(self, mode: str, dt: float = 0.05) -> None:
        self.mode = mode
        self.dt = dt
        self.time_limit = PACMAN_TIME_LIMIT
        self.time_remaining = self.time_limit
        self.score = 0
        self.game_over = False
        self.victory = False
        self.pacman = PacmanState()
        self.pellet_manager = PelletManager()
        self.ghost_manager = GhostManager()
        self.keys_down: set[str] = set()
        self.last_update_time = time.perf_counter()
        self.current_target_id = ""

        self.pellet_manager.reset()
        self.ghost_manager.reset()

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title(f"Phase 2 Pac-Man ({self.mode})")
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.fig.canvas.mpl_connect("key_release_event", self._on_key_release)

        self.pacman_patch = Circle((self.pacman.x, self.pacman.y), SHARK_RADIUS, color="#f5c518")
        self.ghost_patch = Circle((0.0, 0.0), self.ghost_manager.ghost.radius, color="#d7263d")
        self.pellet_scatter = self.ax.scatter([], [], s=40, c="#f5c518")
        self.lidar_lines = [self.ax.plot([], [], color="#5dade2", alpha=0.35, linewidth=1.0)[0] for _ in range(9)]
        self.status_text = self.ax.text(
            PACMAN_WORLD_BOUNDS.x_min + 0.5,
            PACMAN_WORLD_BOUNDS.y_max - 0.5,
            "",
            ha="left",
            va="top",
            fontsize=11,
            family="monospace",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )

        self.ax.add_patch(self.pacman_patch)
        self.ax.add_patch(self.ghost_patch)
        self._setup_axes()

    def _setup_axes(self) -> None:
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(PACMAN_WORLD_BOUNDS.x_min, PACMAN_WORLD_BOUNDS.x_max)
        self.ax.set_ylim(PACMAN_WORLD_BOUNDS.y_min, PACMAN_WORLD_BOUNDS.y_max)
        self.ax.set_title("Phase 2 Pac-Man Simulator")
        self.ax.set_facecolor("#f7f7f7")
        self.ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)

        wall_x = [
            PACMAN_WORLD_BOUNDS.x_min,
            PACMAN_WORLD_BOUNDS.x_max,
            PACMAN_WORLD_BOUNDS.x_max,
            PACMAN_WORLD_BOUNDS.x_min,
            PACMAN_WORLD_BOUNDS.x_min,
        ]
        wall_y = [
            PACMAN_WORLD_BOUNDS.y_min,
            PACMAN_WORLD_BOUNDS.y_min,
            PACMAN_WORLD_BOUNDS.y_max,
            PACMAN_WORLD_BOUNDS.y_max,
            PACMAN_WORLD_BOUNDS.y_min,
        ]
        self.ax.plot(wall_x, wall_y, color="#2c3e50", linewidth=2.0)

        waypoint_x = [point[0] for point in PACMAN_GHOST_WAYPOINTS] + [PACMAN_GHOST_WAYPOINTS[0][0]]
        waypoint_y = [point[1] for point in PACMAN_GHOST_WAYPOINTS] + [PACMAN_GHOST_WAYPOINTS[0][1]]
        self.ax.plot(waypoint_x, waypoint_y, color="#d7263d", alpha=0.2, linestyle=":")

    def _on_key_press(self, event) -> None:
        if event.key:
            self.keys_down.add(event.key.lower())
            if event.key.lower() == "r":
                self.reset()

    def _on_key_release(self, event) -> None:
        if event.key:
            self.keys_down.discard(event.key.lower())

    def reset(self) -> None:
        self.pacman = PacmanState()
        self.pellet_manager.reset()
        self.ghost_manager.reset()
        self.time_remaining = self.time_limit
        self.score = 0
        self.game_over = False
        self.victory = False
        self.current_target_id = ""
        self.last_update_time = time.perf_counter()

    def run(self) -> None:
        self.anim = FuncAnimation(self.fig, self._update_frame, interval=int(self.dt * 1000), blit=False)
        plt.show()

    def _update_frame(self, _frame_index: int):
        now = time.perf_counter()
        elapsed = min(max(now - self.last_update_time, 0.0), 0.1)
        self.last_update_time = now
        self._step(elapsed if elapsed > 0.0 else self.dt)
        self._render()
        return [self.pacman_patch, self.ghost_patch, self.pellet_scatter, self.status_text, *self.lidar_lines]

    def _step(self, dt: float) -> None:
        if self.game_over or self.victory:
            return

        self.time_remaining = max(0.0, self.time_remaining - dt)
        if self.time_remaining <= 0.0:
            self.game_over = True
            return

        if self.mode == "teleop":
            self._teleop_control()
        else:
            self._auto_control()

        self.pacman.heading += self.pacman.angular_speed * dt
        self.pacman.x += math.cos(self.pacman.heading) * self.pacman.linear_speed * dt
        self.pacman.y += math.sin(self.pacman.heading) * self.pacman.linear_speed * dt
        self.pacman.x, self.pacman.y = clamp_to_world(self.pacman.x, self.pacman.y, SHARK_RADIUS, PACMAN_WORLD_BOUNDS)

        self.ghost_manager.update(dt)

        collected = self.pellet_manager.collect_near(self.pacman.x, self.pacman.y, SHARK_RADIUS)
        self.score += sum(pellet.value for pellet in collected)
        if self.pellet_manager.active_count() == 0:
            self.victory = True

        ghost = self.ghost_manager.ghost
        if distance_xy(self.pacman.x, self.pacman.y, ghost.x, ghost.y) <= SHARK_RADIUS + ghost.radius:
            self.game_over = True

    def _teleop_control(self) -> None:
        linear = 0.0
        angular = 0.0
        if "up" in self.keys_down or "w" in self.keys_down:
            linear += 0.9
        if "down" in self.keys_down or "s" in self.keys_down:
            linear -= 0.6
        if "left" in self.keys_down or "a" in self.keys_down:
            angular += 1.8
        if "right" in self.keys_down or "d" in self.keys_down:
            angular -= 1.8
        self.pacman.linear_speed = linear
        self.pacman.angular_speed = angular

    def _auto_control(self) -> None:
        pellets = self.pellet_manager.active_pellets()
        if not pellets:
            self.pacman.linear_speed = 0.0
            self.pacman.angular_speed = 0.0
            self.current_target_id = ""
            return

        target = min(
            pellets,
            key=lambda pellet: distance_xy(self.pacman.x, self.pacman.y, pellet.x, pellet.y)
            + 0.8 * abs(angle_diff(bearing_xy(self.pacman.x, self.pacman.y, pellet.x, pellet.y), self.pacman.heading)),
        )
        self.current_target_id = target.pellet_id
        target_bearing = angle_diff(bearing_xy(self.pacman.x, self.pacman.y, target.x, target.y), self.pacman.heading)
        scan = self._scan_ranges(31, 4.0)
        obstacle_turn, target_signal_heading, front_clearance = self._segment_and_select_heading(scan, target_bearing, 4.0)

        obstacle_mix = clamp(1.0 - (front_clearance / 1.0), 0.0, 1.0)
        signal_gain = 0.9 * obstacle_mix
        pursuit_heading = angle_diff(
            math.atan2(
                math.sin(target_bearing) + signal_gain * math.sin(target_signal_heading),
                math.cos(target_bearing) + signal_gain * math.cos(target_signal_heading),
            ),
            0.0,
        )
        wall_turn = 1.1 * obstacle_turn

        ghost = self.ghost_manager.ghost
        ghost_distance = distance_xy(self.pacman.x, self.pacman.y, ghost.x, ghost.y)
        ghost_turn = 0.0
        if ghost_distance < 1.4:
            ghost_bearing = angle_diff(bearing_xy(self.pacman.x, self.pacman.y, ghost.x, ghost.y), self.pacman.heading)
            ghost_turn = -1.5 * ghost_bearing

        angular = clamp(1.7 * pursuit_heading + wall_turn + ghost_turn, -2.2, 2.2)
        if front_clearance < 0.42:
            linear = 0.0
            angular = 2.0 if wall_turn >= 0.0 else -2.0
        else:
            heading_scale = clamp(1.0 - 0.45 * abs(pursuit_heading), 0.20, 1.0)
            clearance_scale = clamp(front_clearance / 1.0, 0.20, 1.0)
            linear = clamp(0.9 * heading_scale * clearance_scale, 0.16, 1.0)

        self.pacman.linear_speed = linear
        self.pacman.angular_speed = angular

    def _scan_ranges(self, beams: int, range_max: float) -> list[float]:
        distances: list[float] = []
        for idx in range(beams):
            rel = -math.pi + (2.0 * math.pi * idx / max(1, beams - 1))
            distances.append(self._ray_distance(self.pacman.heading + rel, range_max=range_max))
        return distances

    def _ray_distance(self, angle: float, range_max: float = 4.0) -> float:
        return raycast_world(
            self.pacman.x,
            self.pacman.y,
            angle,
            range_max=range_max,
            bounds=PACMAN_WORLD_BOUNDS,
            obstacles=(),
        )

    def _segment_and_select_heading(self, scan: list[float], target_bearing: float, range_max: float) -> tuple[float, float, float]:
        left_risk = 0.0
        right_risk = 0.0
        left_count = 0
        right_count = 0
        front_clearance = range_max
        best_score = -1.0
        best_heading = target_bearing

        for idx, distance in enumerate(scan):
            rel = -math.pi + (2.0 * math.pi * idx / max(1, len(scan) - 1))
            risk = 1.0 - clamp(distance / range_max, 0.0, 1.0)
            clearance = 1.0 - risk
            alignment = max(0.0, math.cos(angle_diff(rel, target_bearing)))
            forward = max(0.0, math.cos(rel))
            signal = clearance * (0.7 * alignment + 0.3 * forward)
            if signal > best_score:
                best_score = signal
                best_heading = rel
            if abs(rel) <= 0.35:
                front_clearance = min(front_clearance, distance)
            if 0.25 <= rel <= 1.20:
                left_risk += risk
                left_count += 1
            if -1.20 <= rel <= -0.25:
                right_risk += risk
                right_count += 1

        obstacle_turn = (right_risk / max(1, right_count)) - (left_risk / max(1, left_count))
        return obstacle_turn, best_heading, front_clearance

    def _render(self) -> None:
        active = self.pellet_manager.active_pellets()
        self.pellet_scatter.set_offsets([(pellet.x, pellet.y) for pellet in active] if active else [])

        self.pacman_patch.center = (self.pacman.x, self.pacman.y)
        ghost = self.ghost_manager.ghost
        self.ghost_patch.center = (ghost.x, ghost.y)

        lidar_angles = [-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9, 1.2, -1.2]
        for line, offset in zip(self.lidar_lines, lidar_angles):
            distance = self._ray_distance(self.pacman.heading + offset)
            end_x = self.pacman.x + math.cos(self.pacman.heading + offset) * distance
            end_y = self.pacman.y + math.sin(self.pacman.heading + offset) * distance
            line.set_data([self.pacman.x, end_x], [self.pacman.y, end_y])

        status = [
            f"mode={self.mode}",
            f"score={self.score}",
            f"time={self.time_remaining:.1f}s",
            f"target={self.current_target_id or '-'}",
            f"collected={len(self.pellet_manager.pellets) - self.pellet_manager.active_count()}",
            f"remaining={self.pellet_manager.active_count()}",
            "controls=WASD / arrows, R reset" if self.mode == "teleop" else "auto controller active",
        ]
        if self.victory:
            status.append("VICTORY")
        elif self.game_over:
            status.append("GAME OVER")
        self.status_text.set_text("\n".join(status))


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal matplotlib Pac-Man simulator")
    parser.add_argument("--mode", choices=("teleop", "auto"), default="teleop")
    parser.add_argument("--dt", type=float, default=0.05)
    args = parser.parse_args()

    sim = MatplotlibPacmanSim(mode=args.mode, dt=args.dt)
    sim.run()


if __name__ == "__main__":
    main()
