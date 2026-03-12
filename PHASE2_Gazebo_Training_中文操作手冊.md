# Phase 2 Matplotlib Pac-Man 中文操作手冊

Phase 2 現在改成純 `matplotlib` 2D 模擬器，不再以 Gazebo 當主路線。

## 1. 需求

```bash
sudo apt install python3-matplotlib
```

## 2. 建置

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase2
source install/setup.bash
```

## 3. Teleop 模式

```bash
ros2 run enpm690_hw3_phase2 pacman_matplotlib_sim -- --mode teleop
```

控制方式：

- `W` / `Up`：前進
- `S` / `Down`：後退
- `A` / `Left`：左轉
- `D` / `Right`：右轉
- `R`：重設

## 4. Auto 模式

```bash
ros2 run enpm690_hw3_phase2 pacman_matplotlib_sim -- --mode auto
```

auto 會：

- 朝最近 pellet 前進
- 用簡單 raycast 模擬 LiDAR 與牆面距離
- 靠近 ghost 時做簡單避讓

## 5. 規則

- 吃到 pellet：`+1`
- 碰到 ghost：遊戲結束
- 全部 pellets 吃完：勝利
- 30 秒到：結束

## 6. 備註

- Phase 1 保持原本的 ROS 2 + Gazebo 路線不變
- 舊的 Phase 2 Gazebo 檔案目前只視為 legacy reference，不再是主展示流程
