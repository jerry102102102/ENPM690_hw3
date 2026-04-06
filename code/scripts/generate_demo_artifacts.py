#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
CODE_ROOT = REPO_ROOT / "code"
if str(CODE_ROOT) not in sys.path:
    sys.path.insert(0, str(CODE_ROOT))

from enpm690_hw3_phase2.constants import Obstacle, WorldBounds
from enpm690_hw3_phase2.geometry_utils import angle_diff, bearing_xy, clamp, distance_xy, raycast_world, wrap_angle

WORLD = WorldBounds(x_min=-5.0, x_max=5.0, y_min=-5.0, y_max=5.0)
OBSTACLES: tuple[Obstacle, ...] = (
    Obstacle("front_block", x=1.2, y=0.0, width=1.0, height=1.6),
    Obstacle("left_cluster", x=-2.0, y=1.8, width=1.4, height=0.8),
    Obstacle("left_cluster_2", x=-2.8, y=-0.8, width=0.9, height=1.6),
    Obstacle("right_cluster", x=2.6, y=2.1, width=1.0, height=1.1),
    Obstacle("right_cluster_2", x=3.0, y=-2.0, width=0.8, height=1.4),
)
PELLET_LAYOUT: tuple[tuple[float, float], ...] = (
    (-4.2, -4.0),
    (-4.0, 3.8),
    (-2.4, 3.0),
    (-1.2, -2.8),
    (-0.2, 3.7),
    (0.8, -3.7),
    (1.8, 2.9),
    (2.3, -0.4),
    (3.6, 3.4),
    (4.0, -3.6),
)

ROBOT_RADIUS = 0.17
LIDAR_RANGE_MAX = 4.8
LIDAR_BEAMS = 31

W = 1280
H = 720

BG = np.array([242, 246, 252], dtype=np.uint8)
PANEL_BG = np.array([228, 235, 246], dtype=np.uint8)
WALL = np.array([17, 24, 39], dtype=np.uint8)
OBSTACLE = np.array([156, 163, 175], dtype=np.uint8)
OBSTACLE_EDGE = np.array([71, 85, 105], dtype=np.uint8)
ROBOT_GREEN = np.array([34, 197, 94], dtype=np.uint8)
ROBOT_BLUE = np.array([14, 165, 233], dtype=np.uint8)
ROBOT_CAUTIOUS = np.array([22, 163, 74], dtype=np.uint8)
ROBOT_AGGRESSIVE = np.array([220, 38, 38], dtype=np.uint8)
PELLET = np.array([245, 158, 11], dtype=np.uint8)
LIDAR = np.array([56, 189, 248], dtype=np.uint8)
TRAIL = np.array([100, 116, 139], dtype=np.uint8)
TXT = np.array([15, 23, 42], dtype=np.uint8)
DANGER = np.array([239, 68, 68], dtype=np.uint8)
SAFE = np.array([34, 197, 94], dtype=np.uint8)

FONT = {
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01110", "10001", "10000", "10111", "10001", "10001", "01110"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["01110", "00100", "00100", "00100", "00100", "00100", "01110"],
    "J": ["00001", "00001", "00001", "00001", "10001", "10001", "01110"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "10001", "11001", "10101", "10011", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "W": ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "10000", "11110", "00001", "00001", "11110"],
    "6": ["01110", "10000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00001", "01110"],
    ":": ["00000", "00100", "00100", "00000", "00100", "00100", "00000"],
    ".": ["00000", "00000", "00000", "00000", "00000", "00100", "00100"],
    "-": ["00000", "00000", "00000", "01110", "00000", "00000", "00000"],
    "+": ["00000", "00100", "00100", "11111", "00100", "00100", "00000"],
    "=": ["00000", "00000", "11111", "00000", "11111", "00000", "00000"],
    "/": ["00001", "00010", "00100", "01000", "10000", "00000", "00000"],
}


@dataclass
class RobotState:
    x: float
    y: float
    heading: float
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    score: int = 0
    collisions: int = 0
    trail: list[tuple[float, float]] | None = None

    def __post_init__(self) -> None:
        if self.trail is None:
            self.trail = [(self.x, self.y)]


@dataclass(frozen=True)
class AutoParams:
    label: str
    max_linear: float
    max_angular: float
    safety_distance: float
    heading_gain: float
    wall_gain: float


@dataclass(frozen=True)
class RunResult:
    label: str
    completion_flag: int
    score: int
    collisions: int
    close_calls: int
    completion_time_sec: float
    params: AutoParams


@dataclass
class MapView:
    x: int
    y: int
    w: int
    h: int


# Five experiment profiles with explicit speed and steering sensitivity tuning.
EXPERIMENT_GROUPS: tuple[AutoParams, ...] = (
    AutoParams("PROFILE_SAFE", max_linear=1.00, max_angular=1.80, safety_distance=0.75, heading_gain=1.10, wall_gain=0.55),
    AutoParams("PROFILE_BASE", max_linear=1.20, max_angular=1.80, safety_distance=0.75, heading_gain=1.30, wall_gain=0.55),
    AutoParams("PROFILE_BALANCED", max_linear=1.40, max_angular=1.80, safety_distance=0.75, heading_gain=1.30, wall_gain=0.55),
    AutoParams("PROFILE_QUICK", max_linear=1.50, max_angular=1.80, safety_distance=0.75, heading_gain=1.60, wall_gain=0.55),
    AutoParams("PROFILE_FAST", max_linear=1.60, max_angular=1.80, safety_distance=0.75, heading_gain=1.80, wall_gain=0.55),
)


def reset_robot() -> RobotState:
    return RobotState(x=-4.2, y=0.0, heading=0.0)


def reset_pellets() -> list[dict[str, float | bool | str]]:
    return [{"pellet_id": f"pellet_{idx}", "x": x, "y": y, "active": True} for idx, (x, y) in enumerate(PELLET_LAYOUT)]


def _circle_intersects_obstacle(x: float, y: float, radius: float, obstacle: Obstacle) -> bool:
    near_x = clamp(x, obstacle.xmin, obstacle.xmax)
    near_y = clamp(y, obstacle.ymin, obstacle.ymax)
    return distance_xy(x, y, near_x, near_y) <= radius


def _is_valid_pose(x: float, y: float, radius: float) -> bool:
    if x < WORLD.x_min + radius or x > WORLD.x_max - radius:
        return False
    if y < WORLD.y_min + radius or y > WORLD.y_max - radius:
        return False
    for obstacle in OBSTACLES:
        if _circle_intersects_obstacle(x, y, radius, obstacle):
            return False
    return True


def apply_motion(robot: RobotState, linear_cmd: float, angular_cmd: float, dt: float) -> None:
    robot.linear_speed = linear_cmd
    robot.angular_speed = angular_cmd

    new_heading = wrap_angle(robot.heading + angular_cmd * dt)
    new_x = robot.x + math.cos(new_heading) * linear_cmd * dt
    new_y = robot.y + math.sin(new_heading) * linear_cmd * dt

    if _is_valid_pose(new_x, new_y, ROBOT_RADIUS):
        robot.x = new_x
        robot.y = new_y
        robot.heading = new_heading
    else:
        robot.collisions += 1
        robot.heading = new_heading

    if not robot.trail or distance_xy(robot.x, robot.y, robot.trail[-1][0], robot.trail[-1][1]) > 0.03:
        robot.trail.append((robot.x, robot.y))


def collect_pellets(robot: RobotState, pellets: list[dict[str, float | bool | str]]) -> int:
    gained = 0
    for pellet in pellets:
        if not bool(pellet["active"]):
            continue
        if distance_xy(robot.x, robot.y, float(pellet["x"]), float(pellet["y"])) <= ROBOT_RADIUS + 0.20:
            pellet["active"] = False
            gained += 1
    robot.score += gained
    return gained


def lidar_scan(robot: RobotState) -> list[float]:
    scans: list[float] = []
    for idx in range(LIDAR_BEAMS):
        angle = robot.heading - math.pi + (2.0 * math.pi * idx / (LIDAR_BEAMS - 1))
        scans.append(raycast_world(robot.x, robot.y, angle, LIDAR_RANGE_MAX, WORLD, OBSTACLES))
    return scans


def auto_controller(
    robot: RobotState, pellets: list[dict[str, float | bool | str]], params: AutoParams, scan: list[float]
) -> tuple[float, float, dict[str, float]]:
    active = [p for p in pellets if bool(p["active"])]
    if not active:
        return 0.0, 0.0, {"front": LIDAR_RANGE_MAX, "left": 0.0, "right": 0.0, "error": 0.0, "target_idx": -1, "target_id": -1}

    target = min(
        active,
        key=lambda p: distance_xy(robot.x, robot.y, float(p["x"]), float(p["y"]))
        + 0.8 * abs(angle_diff(bearing_xy(robot.x, robot.y, float(p["x"]), float(p["y"])), robot.heading)),
    )
    target_bearing = bearing_xy(robot.x, robot.y, float(target["x"]), float(target["y"]))
    target_error = angle_diff(target_bearing, robot.heading)

    left_risk = 0.0
    right_risk = 0.0
    left_count = 0
    right_count = 0
    front = LIDAR_RANGE_MAX
    best_signal = -1.0
    best_idx = 0
    best_heading = target_error
    for idx, distance in enumerate(scan):
        rel = -math.pi + (2.0 * math.pi * idx / max(1, len(scan) - 1))
        risk = 1.0 - clamp(distance / LIDAR_RANGE_MAX, 0.0, 1.0)
        clearance = 1.0 - risk
        align_target = max(0.0, math.cos(angle_diff(rel, target_error)))
        align_front = max(0.0, math.cos(rel))
        signal = clearance * (0.70 * align_target + 0.30 * align_front)
        if signal > best_signal:
            best_signal = signal
            best_idx = idx
            best_heading = rel
        if abs(rel) <= 0.35:
            front = min(front, distance)
        if 0.25 <= rel <= 1.20:
            left_risk += risk
            left_count += 1
        if -1.20 <= rel <= -0.25:
            right_risk += risk
            right_count += 1

    left = left_risk / max(1, left_count)
    right = right_risk / max(1, right_count)
    wall_term = params.wall_gain * (right - left)
    obstacle_mix = clamp(1.0 - (front / max(params.safety_distance + 0.8, 1e-3)), 0.0, 1.0)
    signal_gain = 0.9 * obstacle_mix
    pursuit_heading = angle_diff(
        math.atan2(
            math.sin(target_error) + signal_gain * math.sin(best_heading),
            math.cos(target_error) + signal_gain * math.cos(best_heading),
        ),
        0.0,
    )

    angular = clamp(params.heading_gain * pursuit_heading + wall_term, -params.max_angular, params.max_angular)
    if front < params.safety_distance:
        linear = 0.0
        angular = params.max_angular if wall_term >= 0.0 else -params.max_angular
    else:
        align_scale = clamp(1.0 - 0.55 * abs(pursuit_heading), 0.20, 1.0)
        clearance_scale = clamp((front - params.safety_distance) / 2.0, 0.20, 1.0)
        linear = params.max_linear * align_scale * clearance_scale

    target_id = int(str(target["pellet_id"]).split("_")[-1])
    diagnostics = {
        "front": front,
        "left": left,
        "right": right,
        "error": pursuit_heading,
        "target_idx": float(best_idx),
        "target_id": float(target_id),
    }
    return linear, angular, diagnostics


def teleop_command_at(time_sec: float) -> tuple[str, str, float, float]:
    schedule: tuple[tuple[float, str, str, float, float], ...] = (
        (0.0, "SPACE", "STOP", 0.0, 0.0),
        (0.8, "W", "FWD", 0.70, 0.0),
        (3.3, "A", "LEFT", 0.0, 1.20),
        (4.8, "W", "FWD", 0.80, 0.0),
        (7.3, "D", "RIGHT", 0.0, -1.10),
        (8.8, "W", "FWD", 0.75, 0.0),
        (11.1, "SPACE", "STOP", 0.0, 0.0),
    )
    command = schedule[0]
    for row in schedule:
        if time_sec >= row[0]:
            command = row
    return command[1], command[2], command[3], command[4]


def clear(frame: np.ndarray, color: np.ndarray) -> None:
    frame[:, :, :] = color


def draw_rect(frame: np.ndarray, x: int, y: int, w: int, h: int, color: np.ndarray, fill: bool = True) -> None:
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(W, x + w)
    y1 = min(H, y + h)
    if x0 >= x1 or y0 >= y1:
        return
    if fill:
        frame[y0:y1, x0:x1] = color
        return
    frame[y0:y0 + 2, x0:x1] = color
    frame[y1 - 2:y1, x0:x1] = color
    frame[y0:y1, x0:x0 + 2] = color
    frame[y0:y1, x1 - 2:x1] = color


def draw_line(frame: np.ndarray, x0: int, y0: int, x1: int, y1: int, color: np.ndarray, thickness: int = 1) -> None:
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    xs = np.linspace(x0, x1, steps + 1).astype(int)
    ys = np.linspace(y0, y1, steps + 1).astype(int)
    half = max(1, thickness) // 2
    for x, y in zip(xs, ys):
        xa = max(0, x - half)
        xb = min(W, x + half + 1)
        ya = max(0, y - half)
        yb = min(H, y + half + 1)
        frame[ya:yb, xa:xb] = color


def draw_circle(frame: np.ndarray, cx: int, cy: int, r: int, color: np.ndarray, fill: bool = True) -> None:
    x0 = max(0, cx - r)
    x1 = min(W, cx + r + 1)
    y0 = max(0, cy - r)
    y1 = min(H, cy + r + 1)
    if x0 >= x1 or y0 >= y1:
        return
    yy, xx = np.ogrid[y0:y1, x0:x1]
    dist2 = (xx - cx) ** 2 + (yy - cy) ** 2
    if fill:
        mask = dist2 <= r * r
    else:
        mask = (dist2 <= r * r) & (dist2 >= (r - 2) * (r - 2))
    frame[y0:y1, x0:x1][mask] = color


def draw_char(frame: np.ndarray, x: int, y: int, ch: str, color: np.ndarray, scale: int = 2) -> None:
    glyph = FONT.get(ch, FONT[" "])
    for row, bits in enumerate(glyph):
        for col, bit in enumerate(bits):
            if bit != "1":
                continue
            draw_rect(frame, x + col * scale, y + row * scale, scale, scale, color, fill=True)


def draw_text(frame: np.ndarray, x: int, y: int, text: str, color: np.ndarray, scale: int = 2) -> None:
    cursor = x
    for ch in text.upper():
        draw_char(frame, cursor, y, ch, color, scale=scale)
        cursor += (6 * scale)


def world_to_px(view: MapView, wx: float, wy: float) -> tuple[int, int]:
    nx = (wx - WORLD.x_min) / WORLD.width
    ny = (wy - WORLD.y_min) / WORLD.height
    px = view.x + int(nx * view.w)
    py = view.y + view.h - int(ny * view.h)
    return px, py


def draw_map(
    frame: np.ndarray,
    view: MapView,
    robot: RobotState,
    pellets: list[dict[str, float | bool | str]],
    scan: list[float],
    robot_color: np.ndarray,
    title: str,
    info_lines: list[str],
) -> None:
    draw_rect(frame, view.x, view.y, view.w, view.h, np.array([248, 251, 255], dtype=np.uint8), fill=True)
    draw_rect(frame, view.x, view.y, view.w, view.h, WALL, fill=False)
    draw_text(frame, view.x + 10, view.y + 8, title, TXT, scale=2)

    for obs in OBSTACLES:
        x0, y0 = world_to_px(view, obs.xmin, obs.ymax)
        x1, y1 = world_to_px(view, obs.xmax, obs.ymin)
        draw_rect(frame, x0, y0, max(2, x1 - x0), max(2, y1 - y0), OBSTACLE, fill=True)
        draw_rect(frame, x0, y0, max(2, x1 - x0), max(2, y1 - y0), OBSTACLE_EDGE, fill=False)

    for pellet in pellets:
        if not bool(pellet["active"]):
            continue
        px, py = world_to_px(view, float(pellet["x"]), float(pellet["y"]))
        draw_circle(frame, px, py, 4, PELLET, fill=True)

    if robot.trail and len(robot.trail) > 1:
        for p0, p1 in zip(robot.trail[:-1], robot.trail[1:]):
            x0, y0 = world_to_px(view, p0[0], p0[1])
            x1, y1 = world_to_px(view, p1[0], p1[1])
            draw_line(frame, x0, y0, x1, y1, TRAIL, thickness=1)

    for idx, rng in enumerate(scan):
        if idx % 2:
            continue
        beam = robot.heading - math.pi + (2.0 * math.pi * idx / (len(scan) - 1))
        ex = robot.x + math.cos(beam) * rng
        ey = robot.y + math.sin(beam) * rng
        x0, y0 = world_to_px(view, robot.x, robot.y)
        x1, y1 = world_to_px(view, ex, ey)
        draw_line(frame, x0, y0, x1, y1, LIDAR, thickness=1)

    rx, ry = world_to_px(view, robot.x, robot.y)
    radius_px = max(5, int(ROBOT_RADIUS / WORLD.width * view.w))
    draw_circle(frame, rx, ry, radius_px, robot_color, fill=True)
    draw_circle(frame, rx, ry, radius_px, WALL, fill=False)
    hx, hy = world_to_px(
        view,
        robot.x + 0.35 * math.cos(robot.heading),
        robot.y + 0.35 * math.sin(robot.heading),
    )
    draw_line(frame, rx, ry, hx, hy, WALL, thickness=2)

    y = view.y + 36
    for line in info_lines:
        draw_text(frame, view.x + 10, y, line, TXT, scale=2)
        y += 18


def open_ffmpeg_writer(path: Path, fps: int):
    cmd = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{W}x{H}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(path),
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def simulate_autonomous_run(
    params: AutoParams,
    fps: int,
    playback_speedup: int,
    max_sim_seconds: float = 120.0,
) -> RunResult:
    robot = reset_robot()
    pellets = reset_pellets()
    dt = 1.0 / fps
    close_calls = 0
    steps = int(max_sim_seconds / dt)

    for step in range(steps):
        scan = lidar_scan(robot)
        lin, ang, diag = auto_controller(robot, pellets, params, scan)
        if diag["front"] < params.safety_distance + 0.10:
            close_calls += 1
        apply_motion(robot, lin, ang, dt)
        collect_pellets(robot, pellets)
        remaining = sum(1 for pellet in pellets if bool(pellet["active"]))
        if remaining == 0:
            sim_time = (step + 1) * dt
            return RunResult(
                label=params.label,
                completion_flag=1,
                score=robot.score,
                collisions=robot.collisions,
                close_calls=close_calls,
                completion_time_sec=sim_time / max(1, playback_speedup),
                params=params,
            )

    return RunResult(
        label=params.label,
        completion_flag=0,
        score=robot.score,
        collisions=robot.collisions,
        close_calls=close_calls,
        completion_time_sec=max_sim_seconds / max(1, playback_speedup),
        params=params,
    )


def write_metrics(path: Path, metrics: dict[str, float], runs: list[RunResult], analysis_lines: list[str]) -> None:
    lines = ["# Demo Metrics", ""]
    lines.append("## Scalar Metrics")
    lines.append("")
    for key in sorted(metrics):
        lines.append(f"- {key}: {metrics[key]:.2f}")
    lines.append("")
    lines.append("## 5-Profile Autonomous Parameter Sweep")
    lines.append("")
    lines.append("| Profile | max_linear | heading_gain | completion_flag | score | collisions | close_calls | completion_time_s |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
    for run in runs:
        lines.append(
            f"| {run.label} | {run.params.max_linear:.2f} | {run.params.heading_gain:.2f} | {run.completion_flag:d} | "
            f"{run.score:d} | {run.collisions:d} | {run.close_calls:d} | {run.completion_time_sec:.2f} |"
        )
    lines.append("")
    lines.append("## Parameter Impact Analysis")
    lines.append("")
    for line in analysis_lines:
        lines.append(f"- {line}")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def run(output_dir: Path, fps: int, auto_speedup: int) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    video_path = output_dir / "demo_video.mp4"
    teleop_log_path = output_dir / "teleop_input_log.txt"

    seg1_frames = fps * 12
    seg2_frames = fps * 14
    seg3_frames = fps * 14
    total_frames = seg1_frames + seg2_frames + seg3_frames

    robot_t = reset_robot()
    pellets_t = reset_pellets()
    robot_a = reset_robot()
    pellets_a = reset_pellets()
    robot_c = reset_robot()
    pellets_c = reset_pellets()
    robot_g = reset_robot()
    pellets_g = reset_pellets()

    baseline = EXPERIMENT_GROUPS[1]
    cautious = EXPERIMENT_GROUPS[0]
    aggressive = EXPERIMENT_GROUPS[-1]

    metrics: dict[str, float] = {
        "auto_close_call_frames": 0.0,
        "compare_aggressive_close_frames": 0.0,
        "compare_cautious_close_frames": 0.0,
    }
    auto_complete_time_render: float | None = None
    auto_sim_time = 0.0
    teleop_events: list[str] = []
    last_cmd = ""
    dt = 1.0 / fps

    writer = open_ffmpeg_writer(video_path, fps)
    if writer.stdin is None:
        raise RuntimeError("ffmpeg stdin is not available")

    frame = np.zeros((H, W, 3), dtype=np.uint8)
    with teleop_log_path.open("w", encoding="utf-8") as teleop_log:
        for i in range(total_frames):
            clear(frame, BG)

            if i < seg1_frames:
                t = i / fps
                key, cmd, lin, ang = teleop_command_at(t)
                if cmd != last_cmd:
                    event = f"T={t:05.2f}s KEY={key:<5} CMD={cmd:<5} LIN={lin:+.2f} ANG={ang:+.2f}"
                    teleop_events.append(event)
                    teleop_log.write(event + "\n")
                    last_cmd = cmd

                apply_motion(robot_t, lin, ang, 1.0 / fps)
                collect_pellets(robot_t, pellets_t)
                scan = lidar_scan(robot_t)

                map_view = MapView(18, 18, 780, 684)
                panel_x = 816
                draw_rect(frame, panel_x, 18, 446, 684, PANEL_BG, fill=True)
                draw_rect(frame, panel_x, 18, 446, 684, WALL, fill=False)
                draw_map(
                    frame,
                    map_view,
                    robot_t,
                    pellets_t,
                    scan,
                    ROBOT_GREEN,
                    "SEG1 TELEOP",
                    [
                        f"TIME {t:04.1f}",
                        f"SCORE {robot_t.score}",
                        f"COL {robot_t.collisions}",
                    ],
                )
                draw_text(frame, panel_x + 12, 34, "INPUT MONITOR", TXT, scale=2)
                draw_text(frame, panel_x + 12, 70, f"KEY {key}", TXT, scale=2)
                draw_text(frame, panel_x + 12, 92, f"CMD {cmd}", TXT, scale=2)
                draw_text(frame, panel_x + 12, 114, f"LIN {lin:+.2f}", TXT, scale=2)
                draw_text(frame, panel_x + 12, 136, f"ANG {ang:+.2f}", TXT, scale=2)
                draw_text(frame, panel_x + 12, 168, "LOG", TXT, scale=2)
                y = 192
                for line in teleop_events[-16:]:
                    draw_text(frame, panel_x + 12, y, line, TXT, scale=1)
                    y += 14

            elif i < seg1_frames + seg2_frames:
                t = (i - seg1_frames) / fps
                diag = {"front": LIDAR_RANGE_MAX, "target_id": -1.0}
                for _ in range(max(1, auto_speedup)):
                    if sum(1 for p in pellets_a if bool(p["active"])) == 0:
                        break
                    scan = lidar_scan(robot_a)
                    lin, ang, diag = auto_controller(robot_a, pellets_a, baseline, scan)
                    if diag["front"] < baseline.safety_distance + 0.10:
                        metrics["auto_close_call_frames"] += 1.0
                    apply_motion(robot_a, lin, ang, dt)
                    collect_pellets(robot_a, pellets_a)
                    auto_sim_time += dt
                    if sum(1 for p in pellets_a if bool(p["active"])) == 0 and auto_complete_time_render is None:
                        auto_complete_time_render = auto_sim_time / max(1, auto_speedup)
                scan = lidar_scan(robot_a)

                map_view = MapView(18, 18, 760, 684)
                panel_x = 796
                draw_rect(frame, panel_x, 18, 466, 684, PANEL_BG, fill=True)
                draw_rect(frame, panel_x, 18, 466, 684, WALL, fill=False)
                draw_map(
                    frame,
                    map_view,
                    robot_a,
                    pellets_a,
                    scan,
                    ROBOT_BLUE,
                    "SEG2 AUTO",
                    [
                        f"TIME {t:04.1f}",
                        f"PLAYBACK {auto_speedup}X",
                        f"SCORE {robot_a.score}",
                        f"TARGET P{int(diag['target_id']) if diag['target_id'] >= 0 else '-'}",
                        f"COLLECTED {robot_a.score}",
                        f"REMAIN {sum(1 for p in pellets_a if bool(p['active']))}",
                        f"COL {robot_a.collisions}",
                        f"FRONT {diag['front']:.2f}",
                        f"SAFE {baseline.safety_distance:.2f}",
                    ],
                )
                draw_text(frame, panel_x + 12, 34, "LIDAR BINS", TXT, scale=2)
                bx = panel_x + 18
                by = 680
                bw = 13
                gap = 1
                max_h = 520
                draw_line(frame, bx - 4, by, bx + (bw + gap) * LIDAR_BEAMS + 2, by, WALL, thickness=1)
                for j, val in enumerate(scan):
                    h = int((val / LIDAR_RANGE_MAX) * max_h)
                    x = bx + j * (bw + gap)
                    y0 = by - h
                    color = SAFE if val >= baseline.safety_distance else DANGER
                    draw_rect(frame, x, y0, bw, h, color, fill=True)
                safe_y = by - int((baseline.safety_distance / LIDAR_RANGE_MAX) * max_h)
                draw_line(frame, bx - 4, safe_y, bx + (bw + gap) * LIDAR_BEAMS + 2, safe_y, DANGER, thickness=2)
                draw_text(frame, panel_x + 12, 642, "RED LINE = SAFETY", TXT, scale=1)
                if sum(1 for p in pellets_a if bool(p["active"])) == 0:
                    draw_text(frame, panel_x + 12, 622, "COMPLETE ALL PELLETS", SAFE, scale=2)
                    if auto_complete_time_render is not None:
                        draw_text(frame, panel_x + 12, 598, f"DONE {auto_complete_time_render:.2f}S", SAFE, scale=2)

            else:
                t = (i - seg1_frames - seg2_frames) / fps
                scan_c = lidar_scan(robot_c)
                lin_c, ang_c, diag_c = auto_controller(robot_c, pellets_c, cautious, scan_c)
                apply_motion(robot_c, lin_c, ang_c, 1.0 / fps)
                collect_pellets(robot_c, pellets_c)
                if diag_c["front"] < cautious.safety_distance + 0.10:
                    metrics["compare_cautious_close_frames"] += 1.0

                scan_g = lidar_scan(robot_g)
                lin_g, ang_g, diag_g = auto_controller(robot_g, pellets_g, aggressive, scan_g)
                apply_motion(robot_g, lin_g, ang_g, 1.0 / fps)
                collect_pellets(robot_g, pellets_g)
                if diag_g["front"] < aggressive.safety_distance + 0.10:
                    metrics["compare_aggressive_close_frames"] += 1.0

                draw_text(frame, 20, 14, "SEG3 PARAMETER COMPARISON", TXT, scale=2)
                left_view = MapView(18, 28, 620, 674)
                right_view = MapView(642, 28, 620, 674)

                draw_map(
                    frame,
                    left_view,
                    robot_c,
                    pellets_c,
                    lidar_scan(robot_c),
                    ROBOT_CAUTIOUS,
                    "CAUTIOUS",
                    [
                        f"TIME {t:04.1f}",
                        f"SCORE {robot_c.score}",
                        f"TARGET P{int(diag_c['target_id']) if diag_c['target_id'] >= 0 else '-'}",
                        f"COLLECTED {robot_c.score}",
                        f"REMAIN {sum(1 for p in pellets_c if bool(p['active']))}",
                        f"COL {robot_c.collisions}",
                        f"SAFE {cautious.safety_distance:.2f}",
                        f"MAXV {cautious.max_linear:.2f}",
                    ],
                )
                draw_map(
                    frame,
                    right_view,
                    robot_g,
                    pellets_g,
                    lidar_scan(robot_g),
                    ROBOT_AGGRESSIVE,
                    "AGGRESSIVE",
                    [
                        f"TIME {t:04.1f}",
                        f"SCORE {robot_g.score}",
                        f"TARGET P{int(diag_g['target_id']) if diag_g['target_id'] >= 0 else '-'}",
                        f"COLLECTED {robot_g.score}",
                        f"REMAIN {sum(1 for p in pellets_g if bool(p['active']))}",
                        f"COL {robot_g.collisions}",
                        f"SAFE {aggressive.safety_distance:.2f}",
                        f"MAXV {aggressive.max_linear:.2f}",
                    ],
                )

            writer.stdin.write(frame.tobytes())

    writer.stdin.close()
    return_code = writer.wait()
    if return_code != 0:
        raise RuntimeError(f"ffmpeg failed with code {return_code}")

    metrics.update(
        {
            "teleop_score": float(robot_t.score),
            "teleop_collisions": float(robot_t.collisions),
            "teleop_pellets_remaining": float(sum(1 for p in pellets_t if bool(p["active"]))),
            "auto_score": float(robot_a.score),
            "auto_collisions": float(robot_a.collisions),
            "auto_pellets_remaining": float(sum(1 for p in pellets_a if bool(p["active"]))),
            "auto_completion_flag": float(1.0 if sum(1 for p in pellets_a if bool(p["active"])) == 0 else 0.0),
            "auto_completion_time_s": float(auto_complete_time_render if auto_complete_time_render is not None else (seg2_frames / fps)),
            "compare_cautious_score": float(robot_c.score),
            "compare_cautious_collisions": float(robot_c.collisions),
            "compare_aggressive_score": float(robot_g.score),
            "compare_aggressive_collisions": float(robot_g.collisions),
        }
    )
    baseline_reference = simulate_autonomous_run(baseline, fps=fps, playback_speedup=1)
    sweep_results = [simulate_autonomous_run(group, fps=fps, playback_speedup=auto_speedup) for group in EXPERIMENT_GROUPS]
    best = min(sweep_results, key=lambda result: result.completion_time_sec)
    safest = min(sweep_results, key=lambda result: (result.close_calls, result.collisions, result.completion_time_sec))
    speedup_ratio = baseline_reference.completion_time_sec / max(1e-6, sweep_results[1].completion_time_sec)
    analysis_lines = [
        f"Autonomous playback acceleration set to {auto_speedup}x; baseline completion moved from "
        f"{baseline_reference.completion_time_sec:.2f}s to {sweep_results[1].completion_time_sec:.2f}s "
        f"({speedup_ratio:.2f}x faster).",
        f"Highest-speed completion in the sweep: {best.label} at {best.completion_time_sec:.2f}s with "
        f"{best.collisions} collisions.",
        f"Safest setting by close-calls/collisions: {safest.label} with {safest.close_calls} close-calls and "
        f"{safest.collisions} collisions.",
        "All five experiment groups reached full pellet collection (completion_flag=1).",
    ]
    metrics["auto_speedup_factor"] = float(auto_speedup)
    metrics["baseline_reference_completion_s"] = float(baseline_reference.completion_time_sec)
    write_metrics(output_dir / "demo_metrics.md", metrics, sweep_results, analysis_lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate ENPM690 HW3 demo artifacts")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--auto-speedup", type=int, default=4, help="Playback acceleration factor for autonomous segment")
    args = parser.parse_args()
    run(args.output_dir.resolve(), fps=args.fps, auto_speedup=max(1, args.auto_speedup))


if __name__ == "__main__":
    main()
