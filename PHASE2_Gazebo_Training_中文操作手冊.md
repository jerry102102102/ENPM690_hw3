# ENPM690 HW3 Phase 2 Gazebo-backed Training 中文操作手冊

這份文件整理 Phase 2 在 ROS 2 Jazzy + Gazebo Harmonic 環境下的操作方式，重點是：

- Phase 2 teleop play
- Phase 2 baseline auto play
- Phase 2 Gazebo-backed training
- Phase 2 Gazebo-backed evaluation

這份手冊假設你已經有：

- Ubuntu Linux
- ROS 2 Jazzy
- Gazebo Harmonic
- 可以正常執行 `ros2`、`colcon`

## 1. 安裝系統套件

先更新 apt：

```bash
sudo apt update
```

安裝 ROS / Gazebo 相關套件：

```bash
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-simulation-interfaces \
  python3-colcon-common-extensions
```

如果你之後要做 PPO training，還需要 Python 套件：

```bash
source scripts/dev_env.sh
python -m pip install gymnasium stable-baselines3
```

## 2. 進入工作區

```bash
cd /你的路徑/ENPM690_hw3
```

## 3. 載入 ROS 2 環境

每個新 terminal 都先執行：

```bash
source /opt/ros/jazzy/setup.bash
```

## 4. 建置 Phase 1 + Phase 2

因為 Phase 2 會重用 Phase 1 的 robot / launch / topic flow，建議一起 build：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1 enpm690_hw3_phase2
source install/setup.bash
```

## 4.1 為什麼 training / evaluation 不用 `ros2 run`

這個 repo 的正式做法是：

- play / demo 類 ROS runtime node 繼續用 `ros2 launch` / `ros2 run`
- training / evaluation 類 ML scripts 改用 `python -m ...`
- play/auto 的 `game_manager` 和 training env 共用同一套 game core，episode / fish / catch / target / stun 邏輯不再分兩份維護

原因是：

- Ubuntu 24.04 / Python 3.12 有 PEP 668 保護，不應該直接往系統 Python 安裝第三方 ML 套件
- ROS 2 官方文件也建議額外 Python 套件用 virtual environment 管理
- 但 binary install 的 ROS 2 在 `ros2 run` 產生 Python entry script 時，已知可能仍綁到 `/usr/bin/python3`
- 這會讓 script 看不到 `.venv` 裡安裝的 `gymnasium` / `stable-baselines3`

所以這個 repo 把：

- ROS node 保持在 ROS 執行模型
- `train_ppo` / `eval_policy` 改成用目前啟用的 virtualenv interpreter 執行

建議每個新 terminal 先做：

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
```

`scripts/dev_env.sh` 會：

- activate `.venv`
- source `/opt/ros/jazzy/setup.bash` 或 `setup.zsh`
- source workspace `install/setup.bash` 或 `setup.zsh`
- 印出目前使用的 `sys.executable` 和 `sys.version`

另外，`python -m enpm690_hw3_phase2.train_ppo` 與
`python -m enpm690_hw3_phase2.eval_policy` 在啟動時也會先印：

- `sys.executable`
- `sys.version`
- `gymnasium` import 是否成功
- `stable_baselines3` import 是否成功

## 4.2 正確使用方式

最重要的原則只有一個：

- `python -m enpm690_hw3_phase2.train_ppo ...` 不會自動啟動 Gazebo training stack，除非你有加 `--launch-stack`

也就是說，訓練有兩種正確跑法，而且你必須擇一：

### 方式 A：先手動啟動 training stack，再跑 training

Terminal 1：

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
ros2 launch enpm690_hw3_phase2 phase2_train.launch.py
```

Terminal 2：

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
python -m enpm690_hw3_phase2.train_ppo --timesteps 20000 --output-dir artifacts/phase2_ppo
```

### 方式 B：讓 training script 自己啟動 stack

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
python -m enpm690_hw3_phase2.train_ppo --launch-stack --stack-timeout 90 --timesteps 20000 --output-dir artifacts/phase2_ppo
```

目前預設 training 會：

- 使用 visible Gazebo stack，方便直接看場景
- 使用 CPU 跑 PPO

如果你真的要 headless，再額外加 `--headless`。
如果你真的要 GPU，再額外加 `--device cuda`。

另外，Gazebo-backed training 現在允許：

- `/odom` 必須是 fresh
- `/scan` 如果這一步沒有新資料，可以沿用上一筆 lidar

這是因為 Gazebo stepped simulation 下，lidar 發布頻率可能低於每個 RL step 的節奏。

如果你沒有先手動啟動 `phase2_train.launch.py`，又沒有加 `--launch-stack`，那麼 `train_ppo` 在 `check_env()` 或 `reset()` 階段很可能會卡在：

- 等不到新的 `/scan`
- 等不到新的 `/odom`
- 或 timeout 在 Gazebo simulation service

這不是 PPO 邏輯錯誤，而是 training backend 沒有先準備好。

### Evaluation 的正確跑法

evaluation 也同樣遵守這個規則：

- 你可以先手動起 stack，再跑 `python -m enpm690_hw3_phase2.eval_policy ...`
- 或者直接在 eval 指令加 `--launch-stack`

最常用 headless eval：

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
python -m enpm690_hw3_phase2.eval_policy --launch-stack --headless --model artifacts/phase2_ppo/ppo_shark_hunt.zip --episodes 5
```

## 5. Phase 2 Teleop Play

這是人工操作「玩一局」模式，保留 GUI 和 RViz。

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
ros2 run enpm690_hw3_phase1 keyboard_teleop --ros-args -r /cmd_vel:=/cmd_vel_input
```

說明：

- keyboard teleop 不是直接發到 `/cmd_vel`
- 而是先發到 `/cmd_vel_input`
- Phase 2 game manager 會做 scoring / timer / command gating
- fish 的權威狀態仍然在 `game_manager`
- `marker_publisher` 會把 fish / HUD 發到 RViz
- `gazebo_fish_sync` 會訂閱 `/phase2/fish_state_json`，把同一份 fish 狀態同步到 Gazebo entity

這代表在 teleop play 模式下，你應該同時看到：

- RViz 裡的 fish markers 會移動
- Gazebo 裡的 fish entity 也會移動
- 魚被抓到時，Gazebo 裡的 fish 會消失或暫時掉到地板下方
- fish respawn 後會在新位置重新出現

## 6. Phase 2 Baseline Auto Play

這是 rule-based shark baseline 自動玩一局模式。

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py
```

這個模式會啟動：

- Gazebo GUI
- RViz
- shark robot
- game manager
- fish marker / game state
- gazebo fish sync
- baseline auto controller

在這個模式下，fish 邏輯一樣由 `game_manager` 維護，`gazebo_fish_sync` 只負責把 fish 狀態鏡射到 Gazebo，不會把 fish 行為邏輯搬進 Gazebo。

## 7. Phase 2 Headless Training Stack

這是只啟動 Gazebo training backend 的 stack。
不會啟動 GUI、RViz、teleop logger，也不會啟動 play-mode game manager。

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_train.launch.py
```

這個 launch 主要包含：

- Gazebo server-only
- Phase 2 world
- robot spawn
- ROS <-> Gazebo bridge
- robot_state_publisher
- odom TF support

## 8. 檢查 Training Stack 是否真的起來

在另一個 terminal：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list
```

應該至少看到：

- `/scan`
- `/odom`
- `/clock`
- `/cmd_vel`

檢查 `/scan`：

```bash
ros2 topic echo /scan
```

檢查 `/odom`：

```bash
ros2 topic echo /odom
```

檢查 Gazebo simulation service：

```bash
ros2 service list | grep gzserver
```

你應該重點確認至少有這些：

- `/gzserver/reset_simulation`
- `/gzserver/step_simulation`
- `/gzserver/get_simulation_state`
- `/gzserver/set_simulation_state`
- `/gzserver/spawn_entity`
- `/gzserver/delete_entity`
- `/gzserver/get_entities`
- `/gzserver/set_entity_state`

## 9. 用 Gazebo-backed Env 直接訓練

### 方式 A：先手動啟動 training stack，再跑 PPO

先開一個 terminal 啟動 stack：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_train.launch.py
```

再開另一個 terminal 跑 training：

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
python -m enpm690_hw3_phase2.train_ppo --timesteps 20000 --output-dir artifacts/phase2_ppo
```

### 方式 B：讓 train_ppo.py 自己啟動 training stack

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
python -m enpm690_hw3_phase2.train_ppo --launch-stack --timesteps 20000 --output-dir artifacts/phase2_ppo
```

## 10. Training 常用參數

最常用：

```bash
python -m enpm690_hw3_phase2.train_ppo --launch-stack --timesteps 20000 --output-dir artifacts/phase2_ppo
```

如果你要改 timeout：

```bash
python -m enpm690_hw3_phase2.train_ppo --launch-stack --stack-timeout 60 --timesteps 20000
```

如果你要明確指定 headless：

```bash
python -m enpm690_hw3_phase2.train_ppo --launch-stack --headless --timesteps 20000
```

如果你只是 debug，不想走真 Gazebo env，才用 fallback logical env：

```bash
python -m enpm690_hw3_phase2.train_ppo --use-logical-env --timesteps 2000
```

注意：

- `--use-logical-env` 只是 debug fallback
- 正式 Phase 2 training 應該使用 Gazebo-backed env

## 11. Evaluation

### Headless evaluation

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
python -m enpm690_hw3_phase2.eval_policy --launch-stack --headless --model artifacts/phase2_ppo/ppo_shark_hunt.zip --episodes 5
```

### Visible evaluation

這會啟 visible Gazebo stack，讓 policy 實際玩一局：

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
python -m enpm690_hw3_phase2.eval_policy --launch-stack --no-headless --model artifacts/phase2_ppo/ppo_shark_hunt.zip --episodes 3
```

## 12. 直接啟動 Visible Eval Stack

如果你只想先看 visible evaluation stack 能不能正常起來：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_eval.launch.py
```

這個 launch 會起：

- visible Gazebo
- shark robot
- bridge
- robot_state_publisher
- odom TF support
- RViz

## 13. 訓練輸出檔案

預設輸出會在：

```bash
artifacts/phase2_ppo/
```

通常你應該會看到：

- `ppo_shark_hunt.zip`
- `train_summary.json`
- `tensorboard/`

## 14. 評估輸出檔案

預設會寫到：

```bash
artifacts/phase2_eval_summary.json
```

## 15. 檢查 PPO 模型是否真的產生

```bash
ls artifacts/phase2_ppo
```

## 16. 檢查 fish entity / Gazebo entity

如果你想確認 fish entity 有沒有真的被 spawn 到 Gazebo，或 play/demo 模式下 fish 有沒有被持續同步：

```bash
ros2 service list | grep entity
```

如果你的系統有對應 CLI 或 service call helper，也可以查 `/gzserver/get_entities`。

play / demo 模式下，`gazebo_fish_sync` 會吃 `/phase2/fish_state_json`，然後透過 Gazebo simulation interface 做這些事：

- spawn fish entity
- set fish entity state
- 必要時查現有 entity

Training mode 不依賴這個 play-mode node；Gazebo-backed env 仍然走它自己在 `gazebo_env.py` 內的 fish sync 路徑。

如果你在 `phase2_play_teleop.launch.py` 或 `phase2_play_auto.launch.py` 裡看不到 Gazebo fish 移動，可以先檢查：

- `ros2 run enpm690_hw3_phase2 gazebo_fish_sync` 是否可解析
- launch 後 `gazebo_fish_sync` node 是否真的有起來
- `/phase2/fish_state_json` 是否有持續更新

至少在程式設計上，Gazebo-backed env 會另外用這些 service：

- spawn fish entity
- set fish entity state
- reset simulation
- step simulation

## 17. 最小 Training 驗證流程

建議你第一次回去驗證照這個順序：

1. 先確認 Phase 1 還能跑
2. 再確認 `phase2_play_teleop.launch.py` 還能玩
3. 再確認 `phase2_play_auto.launch.py` 還能自動玩
4. 再確認 `phase2_train.launch.py` 起得來
5. 用 `ros2 topic echo /scan` 和 `/odom` 確認 training stack 有資料
6. 最後再跑 `python -m enpm690_hw3_phase2.train_ppo`

最小命令如下：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1 enpm690_hw3_phase2
source install/setup.bash
ros2 launch enpm690_hw3_phase2 phase2_train.launch.py
```

另開一個 terminal：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /scan
```

再開第三個 terminal：

```bash
cd /你的路徑/ENPM690_hw3
source scripts/dev_env.sh
python -m enpm690_hw3_phase2.train_ppo --timesteps 20000 --output-dir artifacts/phase2_ppo
```

## 18. 常見問題優先檢查

### 1. `ros2 launch ... phase2_train.launch.py` 起不來

先檢查：

```bash
ros2 pkg list | grep enpm690
ros2 pkg list | grep ros_gz
```

### 2. 沒有 `/scan` 或 `/odom`

檢查：

```bash
ros2 topic list
ros2 node list
```

通常優先看：

- bridge 有沒有起來
- robot 有沒有真的 spawn
- Gazebo server 有沒有正常啟動

### 3. training script 說 service not ready

先檢查：

```bash
ros2 service list | grep gzserver
```

如果 service 還沒全部起來，可以把 timeout 拉長：

```bash
python -m enpm690_hw3_phase2.train_ppo --launch-stack --stack-timeout 90 --timesteps 20000
```

### 4. `simulation_interfaces` 找不到

安裝：

```bash
sudo apt install -y ros-jazzy-simulation-interfaces
```

### 5. Python RL 套件找不到

```bash
pip install gymnasium stable-baselines3
```

## 19. 你最常會直接用到的指令

### Build

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1 enpm690_hw3_phase2
source install/setup.bash
```

### Teleop play

```bash
ros2 launch enpm690_hw3_phase2 phase2_play_teleop.launch.py
```

### Keyboard teleop

```bash
ros2 run enpm690_hw3_phase1 keyboard_teleop --ros-args -r /cmd_vel:=/cmd_vel_input
```

### Auto play

```bash
ros2 launch enpm690_hw3_phase2 phase2_play_auto.launch.py
```

### Training stack

```bash
ros2 launch enpm690_hw3_phase2 phase2_train.launch.py
```

### Gazebo-backed PPO train

```bash
python -m enpm690_hw3_phase2.train_ppo --launch-stack --timesteps 20000 --output-dir artifacts/phase2_ppo
```

### Gazebo-backed headless eval

```bash
python -m enpm690_hw3_phase2.eval_policy --launch-stack --headless --model artifacts/phase2_ppo/ppo_shark_hunt.zip --episodes 5
```

### Gazebo-backed visible eval

```bash
python -m enpm690_hw3_phase2.eval_policy --launch-stack --no-headless --model artifacts/phase2_ppo/ppo_shark_hunt.zip --episodes 3
```

如果你要，我下一步可以再幫你補一份更短的「Phase 2 demo / train 一頁版指令卡」，只保留最必要的命令。 
