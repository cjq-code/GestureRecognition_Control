# catkin_ws 操作指南

> **ROS1 Noetic** 综合仿真工作空间，涵盖差速底盘、TurtleBot3、机械臂、体感遥控与导航建图。

---

## 1. 环境要求

| 项目 | 版本/说明 |
|------|----------|
| 操作系统 | Ubuntu 20.04 (Focal Fossa) |
| ROS | ROS1 Noetic |
| Python | 3.8+ |
| Gazebo | 11.x |

### 1.1 基础依赖安装

```bash
# ROS Noetic 基础（若未安装）
sudo apt update
sudo apt install ros-noetic-desktop-full

#  TurtleBot3 与建图/导航依赖
sudo apt install \
  ros-noetic-turtlebot3-gazebo \
  ros-noetic-turtlebot3-teleop \
  ros-noetic-turtlebot3-slam \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-gmapping \
  ros-noetic-move-base \
  ros-noetic-cv-bridge \
  ros-noetic-usb-cam \
  ros-noetic-xacro

# 体感遥控 Python 依赖（必须使用系统 Python 3.8）
/usr/bin/python3 -m pip install --user -r ~/catkin_ws/src/handcontrol/requirements.txt
```

### 1.2 环境变量（建议写入 `~/.bashrc`）

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# TurtleBot3 默认模型
export TURTLEBOT3_MODEL=waffle_pi
```

---

## 2. 编译工作空间

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 若 `turtlebot3_manipulation` 中部分子包报 MoveIt 依赖错误，说明系统未安装 `ros-noetic-moveit*`；这些子包已放置 `CATKIN_IGNORE`，不影响核心功能编译。

---

## 3. 功能包速查

| 包名 | 说明 | 主要用途 |
|------|------|----------|
| `course_pkg` | 自定义导航/仿真/体感集成包 | RViz/Gazebo 启动、SLAM、避障、体感遥控集成 |
| `handcontrol` | MediaPipe 体感遥控 | 摄像头姿态识别 → 发布 `/cmd_vel_body` |
| `handcontrol_sim` | 体感仿真辅助 | 预留扩展包 |
| `turtlebot3` | 官方 TurtleBot3 | 底盘描述、 bringup、键盘遥控、SLAM、导航 |
| `turtlebot3_manipulation` | TB3 + OpenMANIPULATOR-X | 机械臂描述、MoveIt 配置（需 apt 装 moveit） |
| `turtlebot3_manipulation_simulations` | 机械臂 Gazebo 仿真 | 带机械臂的 TB3 仿真环境 |
| `turtlebot3_msgs` | 官方消息定义 | 状态、传感器、动作消息 |
| `Gazebo-Differential-Drive` | 差速轮演示包 | 矩形路径跟踪、翻转行驶 |

---

## 4. 快速启动

### 4.1 仅查看 course_pkg 小车模型（RViz）

```bash
roslaunch course_pkg demo.launch
```

### 4.2 course_pkg 小车 + Gazebo + SLAM 建图

```bash
roslaunch course_pkg demo1.launch
```
- 启动 `gmapping`，订阅 `/scan`
- 可用键盘或遥控节点驱动小车，RViz 中实时查看地图

### 4.3 TurtleBot3 官方仿真（键盘遥控）

```bash
# 启动仿真环境
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 新终端 — 键盘遥控
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### 4.4 体感遥控 course_pkg 小车（Gazebo）

> 需连接摄像头（`/dev/video0`）。

```bash
roslaunch course_pkg demo_pose_teleop.launch
```

**控制姿态说明**

| 身体姿态 | 底盘动作 |
|----------|----------|
| 双臂自然下垂 | 停止 (IDLE) |
| 双臂向前上方抬起 | 前进 |
| 双臂向后下方伸展 | 后退 |
| 左臂抬、右臂放 | 原地左转 |
| 右臂抬、左臂放 | 原地右转 |
| 双手举过头顶 | 紧急停止 (EMERGENCY) |

#### 单独启动体感节点（配合其他仿真）

```bash
# 启动 TurtleBot3 仿真
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 新终端 — 启动体感遥控
roslaunch handcontrol handcontrol.launch
```

### 4.5 TurtleBot3 + 机械臂 + 体感遥控（course_pkg 场景）

```bash
roslaunch course_pkg demo_tb3_open_manip_pose_course.launch
```
- 加载 **TB3 Waffle Pi + OpenMANIPULATOR-X**
- 场景：`pose_course.world`（家居房间，含桌椅柜及地面杂物）
- 体感控制底盘移动，机械臂可用 rqt/trajectory 控制

### 4.6 差速轮演示（矩形路径 / 翻转）

```bash
roslaunch Gazebo-Differential-Drive ddrive.launch

# 走矩形路径
roslaunch Gazebo-Differential-Drive ddrive.launch follow_rect:=True

# 翻车继续行驶
roslaunch Gazebo-Differential-Drive ddrive.launch flip_over:=True
```

---

## 5. 实用脚本与工具

### 5.1 查看可用摄像头

```bash
rosrun handcontrol list_usb_cameras.py
```

### 5.2 自动避障（cmd_vel 融合激光）

```bash
rosrun course_pkg cmd_vel_scan_avoid.py
```
- 订阅 `/cmd_vel_body`（体感输出）与 `/scan`
- 检测到障碍物时自动减速/停止，发布安全 `/cmd_vel`

### 5.3 保存地图

```bash
rosrun map_server map_saver -f ~/my_map
```

---

## 6. 常见问题

| 问题 | 解决方法 |
|------|----------|
| `catkin_make` 报 moveit 相关依赖错误 | `turtlebot3_manipulation` 中部分子包已加 `CATKIN_IGNORE`，不影响编译；如需 MoveIt，请 `sudo apt install ros-noetic-moveit-*` 后移除对应 `CATKIN_IGNORE` |
| MediaPipe 安装失败 | 固定版本安装：`/usr/bin/python3 -m pip install mediapipe==0.10.5 --user` |
| Gazebo 模型加载缓慢或失败 | 首次运行需下载模型，或手动拷贝 `course_pkg/gazebo_models/` 到 `~/.gazebo/models/` |
| 摄像头被占用 | 使用 `list_usb_cameras.py` 查看设备号，启动时指定 `camera_index:=1`（或 `-1` 自动探测） |
| 小车原地打滑/抖动 | 降低 `max_linear_speed` / `max_angular_speed`，或调整 `linear_smooth` / `angular_smooth` |

---

## 7. 目录结构速览

```
catkin_ws/
├── build/          # catkin_make 编译产物
├── devel/          # 环境脚本、库、生成的消息头文件
├── src/
│   ├── course_pkg/                     # 自定义导航/仿真/体感集成
│   ├── handcontrol/                    # MediaPipe 体感遥控
│   ├── handcontrol_sim/                # 体感仿真辅助
│   ├── turtlebot3/                     # TB3 官方包
│   ├── turtlebot3_manipulation/        # TB3 + 机械臂
│   ├── turtlebot3_manipulation_simulations/  # 机械臂仿真
│   ├── turtlebot3_msgs/                # 消息定义
│   └── Gazebo-Differential-Drive/      # 差速轮演示
└── gazebo_models_worlds_collection-master/   # 额外模型与世界
```

---

## 8. 下一步扩展

- **导航**: 使用 `demo1.launch` 建图后，配合 `turtlebot3_navigation` 做自主导航 (`amcl` + `move_base`)。
- **机械臂抓取**: 安装 MoveIt 后启用 `turtlebot3_manipulation_moveit_config`，配合感知实现抓取。
- **多机仿真**: 复制 `spawn_model` 节点并修改命名空间，实现多车协同。

---

*本 README 由项目内容自动生成，如需修改请编辑 `~/catkin_ws/README.md`。*
