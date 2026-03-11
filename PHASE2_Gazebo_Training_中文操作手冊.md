# Phase 2 Pac-Man 中文操作手冊

這份文件只說明新的 Phase 2 Pac-Man 版本，不再使用舊的 shark / fish / RL-heavy 主流程。

## 1. 目標

新的 Phase 2 是建立在 Phase 1 已經可正常運作的 stack 上：

- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz_sim` / `ros_gz_bridge`
- RViz2
- TurtleBot3-like robot
- `/cmd_vel`、`/scan`、`/odom`、`/tf`、`/clock`

Phase 2 現在只新增：

- 固定 pellets
- 1 個 waypoint ghost
- score / timer
- RViz markers
- 1 個簡單 auto baseline

## 2. 建置

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1 enpm690_hw3_phase2
source install/setup.bash
```

## 3. 確認 Phase 1 仍可正常運作

```bash
ros2 launch enpm690_hw3_phase1 phase1_bringup.launch.py
```

另一個 terminal：

```bash
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

這條路線不應該被 Phase 2 破壞。

## 4. Phase 2 Teleop Play

Terminal 1：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_play_teleop.launch.py
```

Terminal 2：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

你會看到：

- Gazebo 裡的 Phase 1 obstacle world
- RViz 裡的 pellets marker
- RViz 裡的 ghost marker
- score / time HUD

規則：

- 吃到 pellet：`+1`
- 撞到 ghost：遊戲結束
- 30 秒到：遊戲結束
- 全部 pellets 吃完：勝利

## 5. Phase 2 Auto Play

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py
```

這個 baseline 會：

- 朝最近 pellet 前進
- 用 LiDAR 做避障
- ghost 太近時加入簡單避讓

可調參數：

- `forward_speed`
- `turn_gain`
- `ghost_avoid_gain`

例如：

```bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py \
  forward_speed:=0.30 \
  turn_gain:=1.9 \
  ghost_avoid_gain:=1.8
```

## 6. Topics

- `/phase2/score`
- `/phase2/time_remaining`
- `/phase2/game_state_json`
- `/phase2/pellet_state_json`
- `/phase2/ghost_state_json`
- `/phase2/markers`

## 7. Train Launch

`phase2_train.launch.py` 現在只保留 skeleton，不是主展示路線。

## 8. 備註

舊的 shark / fish / Gazebo training 相關檔案如果還留在 package 中，現在都視為 legacy reference，不再是主執行路線。
