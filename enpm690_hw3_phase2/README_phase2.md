# ENPM690 Homework 3 Phase 2

This package adds the Phase 2 shark-hunt scenario on top of the working Phase 1 stack.

## What Phase 2 adds

- 30 second shark-vs-fish game loop
- rule-based tuna, sardine, and seaweed management
- score, timer, catch, respawn, and stun handling
- baseline autonomous shark controller
- RViz marker visualization for fish and HUD text
- Gymnasium-compatible shark hunting environment
- PPO training and evaluation scripts
- Gazebo-backed training environment for the default RL path

## Package layout

- `launch/`: teleop play, auto play, training, evaluation, and demo launch files
- `config/`: runtime parameters and RL defaults
- `worlds/`: Phase 2 ocean-themed arena
- `models/`: simple fish visuals
- `enpm690_hw3_phase2/`: game logic, ROS nodes, and RL code

## Main topics

- `/phase2/score`
- `/phase2/time_remaining`
- `/phase2/mode`
- `/phase2/catch_event`
- `/phase2/fish_state_json`
- `/phase2/game_state_json`
- `/phase2/markers`
- `/cmd_vel_input`
- `/cmd_vel`

## Teleop play mode

Bring up the environment:

```bash
ros2 launch enpm690_hw3_phase2 phase2_play_teleop.launch.py
```

Run keyboard teleop in a second terminal and remap it into the gated input topic:

```bash
ros2 run enpm690_hw3_phase1 keyboard_teleop --ros-args -r /cmd_vel:=/cmd_vel_input
```

## Autonomous baseline mode

```bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py
```

## Training

Default training now uses the Gazebo-backed environment.
The old Python-only environment remains available only as a debug fallback.
Use a project virtual environment for ML dependencies and run the training/evaluation entry points with
`python -m ...`, not `ros2 run`.
The play/auto `game_manager` path and the training env now share the same core episode state, fish, catch, target, and stun logic.

Why this repo does that:

- Ubuntu 24.04 / Python 3.12 uses PEP 668 protections, so installing third-party ML packages into the system Python is the wrong path.
- ROS 2 documentation recommends virtual environments for extra Python dependencies.
- In binary-installed ROS 2 setups, `ros2 run` can still execute Python console entry scripts with `/usr/bin/python3` instead of the active `.venv` interpreter.
- `python -m enpm690_hw3_phase2.train_ppo` and `python -m enpm690_hw3_phase2.eval_policy` reliably use the currently active virtualenv interpreter.

Install Python RL dependencies first:

```bash
source scripts/dev_env.sh
python -m pip install gymnasium stable-baselines3
```

Train:

```bash
python -m enpm690_hw3_phase2.train_ppo --launch-stack --timesteps 20000 --output-dir artifacts/phase2_ppo
```

Evaluate:

```bash
python -m enpm690_hw3_phase2.eval_policy --launch-stack --headless --model artifacts/phase2_ppo/ppo_shark_hunt.zip --episodes 5
```

Both scripts print startup diagnostics for:

- `sys.executable`
- `sys.version`
- `gymnasium` import status
- `stable_baselines3` import status

The ROS runtime nodes remain standard ROS executables:

- `ros2 launch enpm690_hw3_phase2 phase2_play_teleop.launch.py`
- `ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py`
- `ros2 run enpm690_hw3_phase2 game_manager`
- `ros2 run enpm690_hw3_phase2 gazebo_fish_sync`
- `ros2 run enpm690_hw3_phase2 marker_publisher`
- `ros2 run enpm690_hw3_phase2 shark_auto_controller`
