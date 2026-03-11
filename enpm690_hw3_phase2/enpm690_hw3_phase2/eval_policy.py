#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .runtime_diagnostics import log_python_runtime


def main() -> None:
    log_python_runtime("eval_policy")

    parser = argparse.ArgumentParser(description="Evaluate a saved PPO policy on the Phase 2 shark hunt environment.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--output", type=Path, default=Path("artifacts/phase2_eval_summary.json"))
    parser.add_argument("--launch-stack", action="store_true")
    parser.add_argument("--stack-timeout", type=float, default=45.0)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--use-logical-env", action="store_true")
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
    except ImportError as exc:
        raise SystemExit(
            "stable-baselines3 is required for evaluation. Install it with `pip install stable-baselines3 gymnasium`."
        ) from exc

    from .gazebo_env import GazeboSharkHuntEnv
    from .logical_env import SharkHuntLogicalEnv

    env = None
    try:
        if args.use_logical_env:
            env = SharkHuntLogicalEnv()
            env_name = "logical"
        else:
            env = GazeboSharkHuntEnv(
                launch_stack=args.launch_stack,
                stack_timeout=args.stack_timeout,
                headless=args.headless,
                enable_fish_visuals=not args.headless,
                robot_name="tb3_phase2_train" if args.headless else "tb3_phase2_eval",
                command_topic="/cmd_vel",
            )
            env_name = "gazebo"
        model = PPO.load(str(args.model))

        scores = []
        catch_totals = {"tuna": 0, "sardine": 0, "seaweed": 0}
        collisions = 0
        lengths = []

        for _ in range(args.episodes):
            obs, info = env.reset()
            done = False
            score = 0
            episode_steps = 0
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, step_info = env.step(action)
                done = terminated or truncated
                episode_steps += 1
                score = step_info.get("score", score)
                if step_info.get("collision", False):
                    collisions += 1
            scores.append(score)
            lengths.append(episode_steps)
            for species, count in env.catch_counts.items():
                catch_totals[species] += count

        summary = {
            "episodes": args.episodes,
            "average_score": float(np.mean(scores)) if scores else 0.0,
            "average_episode_length": float(np.mean(lengths)) if lengths else 0.0,
            "collision_count": collisions,
            "average_catches": {species: catch_totals[species] / args.episodes for species in catch_totals},
            "model": str(args.model),
            "environment": env_name,
        }
    finally:
        if env is not None:
            env.close()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
