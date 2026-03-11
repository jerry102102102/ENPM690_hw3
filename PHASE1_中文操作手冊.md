# ENPM690 HW3 Phase 1 中文操作手冊

這份文件整理 Phase 1 環境從安裝、建置到啟動與驗證會用到的主要指令。

適用目標環境：

- Ubuntu Linux
- ROS 2 Jazzy
- Gazebo Harmonic

## 1. 安裝系統套件

先載入 apt 索引：

```bash
sudo apt update
```

安裝 ROS 2 Jazzy 桌面版與本專案需要的核心套件：

```bash
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2
```

如果你還沒有 `colcon`：

```bash
sudo apt install -y python3-colcon-common-extensions
```

## 2. 進入工作區

```bash
cd /你的路徑/ENPM690_hw3
```

如果你的實際路徑和我這裡不同，請替換成你自己的資料夾。

## 3. 載入 ROS 2 環境

每開一個新 terminal，先執行：

```bash
source /opt/ros/jazzy/setup.bash
```

## 4. 建置專案

在專案根目錄執行：

```bash
colcon build --packages-select enpm690_hw3_phase1
```

建置完成後載入 workspace：

```bash
source install/setup.bash
```

如果你要一次做完，可以連續執行：

```bash
source /opt/ros/jazzy/setup.bash
cd /你的路徑/ENPM690_hw3
colcon build --packages-select enpm690_hw3_phase1
source install/setup.bash
```

## 5. 啟動 Phase 1 主系統

Terminal 1：

```bash
source /opt/ros/jazzy/setup.bash
cd /你的路徑/ENPM690_hw3
source install/setup.bash
ros2 launch enpm690_hw3_phase1 phase1_bringup.launch.py
```

這個 launch 會啟動：

- Gazebo Harmonic
- 自訂 obstacle world
- TurtleBot3-like 機器人 spawn
- `ros_gz_bridge`
- `robot_state_publisher`
- `/odom -> /tf` 廣播節點
- teleop log 節點
- RViz2

## 6. 啟動鍵盤遙控

建議另外開一個 terminal，因為 keyboard teleop 需要獨立接管鍵盤輸入。

Terminal 2：

```bash
source /opt/ros/jazzy/setup.bash
cd /你的路徑/ENPM690_hw3
source install/setup.bash
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

## 7. 鍵盤控制方式

在 teleop terminal 內可使用：

```text
w : 前進
x : 後退
a : 左轉
d : 右轉
s : 停止
space : 停止
q : 增加線速度
z : 減少線速度
e : 增加角速度
c : 減少角速度
Ctrl-C : 離開
```

## 8. Topic 檢查指令

確認 ROS topic 是否存在：

```bash
ros2 topic list
```

確認 `/cmd_vel`：

```bash
ros2 topic echo /cmd_vel
```

確認 `/scan`：

```bash
ros2 topic echo /scan
```

確認 `/odom`：

```bash
ros2 topic echo /odom
```

確認 `/clock`：

```bash
ros2 topic echo /clock
```

查看 topic 頻率：

```bash
ros2 topic hz /scan
ros2 topic hz /odom
```

查看 topic 型別：

```bash
ros2 topic type /cmd_vel
ros2 topic type /scan
ros2 topic type /odom
```

## 9. TF 檢查指令

查看 TF 樹：

```bash
ros2 run tf2_tools view_frames
```

如果你只想看某兩個 frame 之間的 transform：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

也可以檢查 LiDAR frame：

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

## 10. RViz / LiDAR 驗證步驟

建議實際照下面順序驗證：

1. 先確認 Gazebo 有正常開起來，機器人成功 spawn。
2. 確認 RViz 有看到 RobotModel、TF、LaserScan、Odometry。
3. 在 teleop terminal 按 `w` 往前。
4. 朝前方障礙物移動，觀察 `/scan` 數值是否改變。
5. 按 `a` 或 `d` 左右旋轉，確認 RViz 中 LaserScan 形狀跟著改變。
6. 確認主 terminal 中 teleop logger 會顯示 `forward`、`turn left`、`turn right`、`stop` 等文字。

## 11. 常用除錯指令

列出 package：

```bash
ros2 pkg list | grep enpm690
```

確認 launch 檔存在：

```bash
ros2 launch enpm690_hw3_phase1 phase1_bringup.launch.py --show-args
```

查看 node：

```bash
ros2 node list
```

查看某個 node 的資訊：

```bash
ros2 node info /teleop_command_logger
```

如果你想看 bridge 節點是否存在：

```bash
ros2 node list | grep bridge
```

## 12. 重新建置的建議流程

如果你改過檔案，建議重新執行：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1
source install/setup.bash
```

## 13. 一次複製用指令

先建置：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1
source install/setup.bash
```

啟動系統：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch enpm690_hw3_phase1 phase1_bringup.launch.py
```

啟動鍵盤控制：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

## 14. 如果啟動失敗，優先檢查

1. 是否真的在 Ubuntu Linux，不是在 macOS 直接跑 ROS 2 Jazzy。
2. `/opt/ros/jazzy/setup.bash` 是否存在。
3. `ros-jazzy-ros-gz-sim` 和 `ros-jazzy-ros-gz-bridge` 是否已安裝。
4. 是否已經執行過 `colcon build`。
5. 每個 terminal 是否都有重新 `source /opt/ros/jazzy/setup.bash` 和 `source install/setup.bash`。
6. Gazebo、RViz2 是否能正常開視窗。

## 15. 建議你回去實機驗證的最小流程

最小驗證流程如下：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select enpm690_hw3_phase1
source install/setup.bash
ros2 launch enpm690_hw3_phase1 phase1_bringup.launch.py
```

再開第二個 terminal：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run enpm690_hw3_phase1 keyboard_teleop
```

再開第三個 terminal 驗證：

```bash
cd /你的路徑/ENPM690_hw3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list
ros2 topic echo /scan
```

如果你要，我下一步可以再幫你補一份更短的「助教 demo 專用 1 頁版指令清單」。
