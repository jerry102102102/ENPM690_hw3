#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

from .runtime_diagnostics import log_python_runtime


def _run_random_smoke(env, steps: int = 20) -> None:
    observation, _ = env.reset()
    for _ in range(steps):
        action = env.action_space.sample()
        observation, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            observation, _ = env.reset()


def _run_scripted_smoke(env, steps: int = 20) -> None:
    observation, _ = env.reset()
    scripted_action = np.array([0.5, 0.0], dtype=np.float32)
    for _ in range(steps):
        observation, reward, terminated, truncated, _ = env.step(scripted_action)
        if terminated or truncated:
            observation, _ = env.reset()


def _format_seconds(seconds: float) -> str:
    total = max(0, int(seconds))
    minutes, sec = divmod(total, 60)
    hours, minutes = divmod(minutes, 60)
    if hours > 0:
        return f"{hours:d}h{minutes:02d}m{sec:02d}s"
    return f"{minutes:02d}m{sec:02d}s"


def main() -> None:
    log_python_runtime("train_ppo")

    parser = argparse.ArgumentParser(description="Train PPO on the Phase 2 shark hunt environment.")
    parser.add_argument("--timesteps", type=int, default=20_000)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/phase2_ppo"))
    parser.add_argument("--launch-stack", action="store_true")
    parser.add_argument("--stack-timeout", type=float, default=45.0)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--device", choices=["cpu", "cuda", "auto"], default="cpu")
    parser.add_argument("--use-logical-env", action="store_true")
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import BaseCallback
        from stable_baselines3.common.env_checker import check_env
    except ImportError as exc:
        raise SystemExit(
            "stable-baselines3 is required for training. Install it with `pip install stable-baselines3 gymnasium`."
        ) from exc

    class ProgressCallback(BaseCallback):
        def __init__(self, total_timesteps: int, report_interval_sec: float = 5.0) -> None:
            super().__init__()
            self.total_timesteps = max(1, total_timesteps)
            self.report_interval_sec = report_interval_sec
            self.start_time = 0.0
            self.last_report_time = 0.0

        def _on_training_start(self) -> None:
            self.start_time = time.time()
            self.last_report_time = self.start_time
            print(
                f"[train_ppo] training started total_timesteps={self.total_timesteps}",
                flush=True,
            )

        def _on_step(self) -> bool:
            now = time.time()
            if now - self.last_report_time < self.report_interval_sec:
                return True

            elapsed = max(now - self.start_time, 1e-6)
            done = self.num_timesteps
            progress = min(100.0, 100.0 * done / self.total_timesteps)
            steps_per_sec = done / elapsed
            remaining = max(0, self.total_timesteps - done)
            eta = remaining / max(steps_per_sec, 1e-6)
            print(
                "[train_ppo] progress "
                f"timesteps={done}/{self.total_timesteps} "
                f"({progress:.1f}%) elapsed={_format_seconds(elapsed)} "
                f"eta={_format_seconds(eta)} fps={steps_per_sec:.1f}",
                flush=True,
            )
            self.last_report_time = now
            return True

    from .gazebo_env import GazeboSharkHuntEnv
    from .logical_env import SharkHuntLogicalEnv

    args.output_dir.mkdir(parents=True, exist_ok=True)
    env = None
    try:
        if args.use_logical_env:
            env = SharkHuntLogicalEnv()
            env_name = "logical"
            print("[train_ppo] environment=logical", flush=True)
        else:
            try:
                if not args.headless and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                    print(
                        "[train_ppo] warning: visible Gazebo requested but DISPLAY/WAYLAND_DISPLAY is unset. "
                        "The Gazebo server may still run, but the GUI window will not appear in this shell session.",
                        flush=True,
                    )
                env = GazeboSharkHuntEnv(
                    launch_stack=args.launch_stack,
                    stack_timeout=args.stack_timeout,
                    headless=args.headless,
                    enable_fish_visuals=False,
                    command_topic="/cmd_vel",
                    launch_log_mode="inherit" if not args.headless else "silent",
                )
            except Exception as exc:
                raise RuntimeError(
                    "Failed to initialize GazeboSharkHuntEnv. "
                    "If diagnostics show /odom updating but /scan not updating, try "
                    "`python -m enpm690_hw3_phase2.train_ppo --launch-stack --no-headless ...` "
                    "to confirm a headless lidar/rendering issue."
                ) from exc
            env_name = "gazebo"
            print(
                f"[train_ppo] environment=gazebo visible={not args.headless} "
                f"launch_stack={args.launch_stack} device={args.device}",
                flush=True,
            )
        check_env(env, warn=True)
        _run_random_smoke(env)
        _run_scripted_smoke(env)
        if isinstance(env, GazeboSharkHuntEnv):
            env.run_motion_smoke_test()

        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            tensorboard_log=str(args.output_dir / "tensorboard"),
            device=args.device,
        )
        model.learn(total_timesteps=args.timesteps, callback=ProgressCallback(args.timesteps))
        model.save(str(args.output_dir / "ppo_shark_hunt"))

        summary = {
            "timesteps": args.timesteps,
            "model_path": str(args.output_dir / "ppo_shark_hunt.zip"),
            "environment": env_name,
        }
    finally:
        if env is not None:
            env.close()

    (args.output_dir / "train_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
