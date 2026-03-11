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

Install Python RL dependencies first:

```bash
pip install gymnasium stable-baselines3
```

Train:

```bash
ros2 run enpm690_hw3_phase2 train_ppo -- --timesteps 20000 --output-dir artifacts/phase2_ppo
```

Evaluate:

```bash
ros2 run enpm690_hw3_phase2 eval_policy -- --model artifacts/phase2_ppo/ppo_shark_hunt.zip --episodes 5
```
