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
| `demo_tb3_keyboard_control.launch` | 共享仿真启动文件 | TB3+机械臂，只启动 Gazebo/控制器/初始机械臂姿态 |
| `key_mouse_teleop.py` | 键鼠 GUI 控制节点 | 鼠标速度盘 + 键盘控制底盘/机械臂/夹爪 |
| `start_key_mouse_control.sh` | 键鼠一键启动脚本 | 自动开 Gazebo + 启动键鼠 GUI |
| `start_body_pose_control.sh` | 体感一键启动脚本 | 自动开 Gazebo + 启动体感底盘控制 |

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

### 4.4 体感控制方案（底盘 + 机械臂）

> 交互原则：左手负责模式与安全，右手负责连续控制。避免使用双臂大幅挥动或腿部动作，降低摄像头裁切、遮挡和误触发的影响。

| 左手动作 | 当前模式 | 右手动作 | 控制结果 |
|----------|----------|----------|----------|
| 左手放下 | 待机 | 任意 | 底盘停止，机械臂保持当前位置 |
| 左手张开并举在肩旁 | 底盘模式 | 右手作为空中摇杆 | 控制底盘移动 |
| 左手握拳并举在肩旁 | 机械臂模式 | 右手位置 + 手势 | 控制机械臂与夹爪 |
| 双手合十或交叉在胸前 | 安全锁定 | 任意 | 立即停车，并解锁下一次模式切换 |

`BASE` 和 `ARM` 不能直接互切。进入底盘模式后，如果要切到机械臂模式，必须先双手合十或交叉进入 `SAFETY`；从机械臂模式切回底盘模式也一样。未经过 `SAFETY` 的互切会被当作待机停止处理，避免手势误识别导致模式跳变。

底盘模式下，右手以右肩附近位置为零点：

| 右手动作 | 底盘动作 |
|----------|----------|
| 右手停在右肩附近死区 | 停止 |
| 右手上移 | 前进 |
| 右手下移 | 后退 |
| 右手左移 | 左转 |
| 右手右移 | 右转 |
| 右手斜向移动 | 前进/后退同时转向 |

机械臂模式下，左手握拳举在肩旁进入 `ARM`，右手以右肩附近位置为零点，用离散动作触发机械臂预设姿态和夹爪：

| 右手动作 | 机械臂/夹爪动作 |
|----------|------------------|
| 右手上移 | 机械臂准备/抬起姿态（同键鼠控制 1） |
| 右手下移 | 机械臂前伸抓取/放下姿态（同键鼠控制 2） |
| 右手握拳 | 夹爪闭合，抓取 |
| 右手张开手掌 | 夹爪张开，释放 |

### 4.5 差速轮演示（矩形路径 / 翻转）

```bash
roslaunch Gazebo-Differential-Drive ddrive.launch

# 走矩形路径
roslaunch Gazebo-Differential-Drive ddrive.launch follow_rect:=True

# 翻车继续行驶
roslaunch Gazebo-Differential-Drive ddrive.launch flip_over:=True
```

### 4.6 TurtleBot3 + 机械臂 键鼠 GUI 控制仿真（无需相机）

```bash
cd ~/catkin_ws
./start_key_mouse_control.sh
```

- 鼠标：在速度盘中按住左键拖拽，向上/下控制前进/后退，向左/右控制转向，松开即停止
- 键盘：**W/A/S/D** 或方向键控制底盘，松开按键停止
- **X/空格**：底盘停止
- **R/U**：急停/解除急停
- **1/2**：机械臂准备姿态/前伸抓取姿态
- **O/C**：夹爪张开/闭合
- 默认 Gazebo 背景为轻量 `simple_course.world`，避免复杂家具模型导致低帧率或崩溃
- 场景包含 4 个约 6.5～8 cm 级地面物块和一个低矮收集架，便于驾驶小车靠近后用夹爪抓取并集中放置
- 物块质量已降低到约 15～18 g，并提高物块/夹爪接触摩擦；夹爪控制器增益和夹爪关节 effort 已提高，降低抓起后滑落的概率
- 低功耗 CPU 如 i3-N305 可关闭 Gazebo 图形窗口，只保留仿真服务和键鼠 GUI：
  ```bash
  GAZEBO_GUI=false ./start_key_mouse_control.sh
  ```

> 如需只启动键鼠 GUI（配合已运行的仿真）：
> ```bash
> roslaunch handcontrol key_mouse_teleop.launch
> ```

### 4.7 TurtleBot3 + 机械臂 体感底盘控制仿真

默认启动完整仿真：Gazebo 图形窗口 + TB3 机械臂场景 + 体感识别调试画面。

```bash
cd ~/catkin_ws
./start_body_pose_control.sh
```

启动脚本会先运行 `course_pkg demo_tb3_keyboard_control.launch`，等待 Gazebo 加载后再运行 `handcontrol.launch`。当前体感控制已接通底盘和机械臂离散预设姿态，不启动键鼠控制器，不启用避障融合节点。

#### 输入源选择

```bash
# 默认：外接 USB 摄像头，等价于 CAMERA_INDEX=0
./start_body_pose_control.sh

# 明确使用 /dev/video0
CAMERA_INDEX=0 ./start_body_pose_control.sh

# 使用已录制视频回放测试
CAMERA_INDEX=1 ./start_body_pose_control.sh

# 使用其他录制视频；必须配合 CAMERA_INDEX=1
CAMERA_INDEX=1 VIDEO_PATH=/home/cjq/catkin_ws/pose_test_videos/pose_test.mp4 ./start_body_pose_control.sh
```

- `CAMERA_INDEX=0`：使用外接 USB 摄像头，当前按 `/dev/video0` 打开
- `CAMERA_INDEX=1`：不打开 USB 摄像头，改用录制视频回放，默认视频为 `/home/cjq/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4`
- `VIDEO_PATH`：只在 `CAMERA_INDEX=1` 时生效；建议使用绝对路径
- 若手动给体感节点传 `pose_test_videos/xxx.mp4` 这种相对路径，节点会优先按 `~/catkin_ws/pose_test_videos/xxx.mp4` 解析，避免 ROS 工作目录变成 `~/.ros` 后找错文件

#### 显示选项

```bash
# 关闭 Gazebo 图形窗口，只保留 gzserver 仿真服务和体感调试画面
GAZEBO_GUI=false ./start_body_pose_control.sh

# 关闭体感识别 OpenCV 调试画面，Gazebo 仍显示
SHOW_DEBUG=false ./start_body_pose_control.sh

# 同时关闭 Gazebo 图形窗口和体感调试画面
GAZEBO_GUI=false SHOW_DEBUG=false ./start_body_pose_control.sh
```

- `GAZEBO_GUI=true|false`：是否显示 Gazebo 图形窗口，默认 `true`
- `SHOW_DEBUG=true|false`：是否显示体感识别画面，默认 `true`
- 启动时脚本会打印实际传给 `roslaunch handcontrol handcontrol.launch` 的参数，便于确认输入源是否正确

#### 控制行为

- 体感节点直接发布 `/cmd_vel`，不经过避障融合节点
- 左手张开举在肩旁进入底盘模式；保持姿势时，右手空中摇杆会持续控制底盘
- 左手握拳举在肩旁进入机械臂模式；右手上移触发准备/抬起姿态，右手下移触发前伸抓取/放下姿态
- 机械臂姿态是离散命令，复用键鼠控制的 `1/2` 两个预设姿态，不做连续关节映射
- 底盘模式和机械臂模式不能直接互切，必须先双手合十或交叉进入安全锁定
- 进入机械臂模式时，`teleop_mapper` 会立即发布零速度，机械臂动作过程中底盘不能继续移动
- 体感底盘最大线速度为 `0.15 m/s`；转向不是比例速度，右手偏左/右超过阈值后固定以 `0.18 rad/s` 慢速旋转，回到中心死区立即停止转向
- 左手放下或未形成有效模式进入 `STANDBY` 时，`teleop_mapper` 立即发布零速度，不保留上一帧速度
- 体感信号超时或丢失时，`teleop_mapper` 也会立即发布零速度
- 双手合十或交叉在胸前进入安全锁定，`teleop_mapper` 立即发布零速度，不走平滑减速

#### 单独启动体感节点

如 Gazebo 已经通过其他终端启动，只想接入体感底盘控制和机械臂离散预设姿态：

```bash
roslaunch handcontrol handcontrol.launch \
  camera_index:=0 \
  video_path:= \
  loop_video:=false \
  show_debug:=true \
  cmd_vel_out_topic:=/cmd_vel \
  use_manip_bridge:=true
```

用录制视频替代摄像头：

```bash
roslaunch handcontrol handcontrol.launch \
  camera_index:=0 \
  video_path:=/home/cjq/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4 \
  loop_video:=true \
  show_debug:=true \
  cmd_vel_out_topic:=/cmd_vel \
  use_manip_bridge:=true
```

#### 常见问题

- `Video file does not exist: /home/cjq/.ros/pose_test_videos/...`：说明使用了旧脚本或手动传了相对路径。重新运行 `./start_body_pose_control.sh`，或把 `video_path` 改成 `/home/cjq/catkin_ws/pose_test_videos/...` 绝对路径。
- `Cannot open camera index 0`：摄像头未接入、被其他程序占用，或设备号不是 0。先运行 `rosrun handcontrol list_usb_cameras.py` 查看可用设备。
- 体感节点退出后底盘仍有速度：当前 `teleop_mapper` 在安全锁定、待机和信号超时时都会发布零速度；若仍异常，检查是否有其他节点同时发布 `/cmd_vel`。

---

## 5. 实用脚本与工具

### 5.1 查看可用摄像头

```bash
rosrun handcontrol list_usb_cameras.py
```

### 5.2 录制体感识别测试视频

```bash
# 默认录制 30 秒，保存到 ~/catkin_ws/pose_test_videos/
rosrun handcontrol record_pose_test_video.py

# 自动选择第一个可用摄像头，录制 60 秒
rosrun handcontrol record_pose_test_video.py --camera-index -1 --duration 60

```

- 默认保存原始摄像头画面，适合作为体感识别回放/标注测试素材
- 预览窗口中按 **空格** 暂停/继续，按 **Q/Esc** 结束录制
- 无图形界面环境可加 `--no-preview`；需要把录制时间信息写进视频时加 `--record-overlay`

#### 回放测试体感动作识别

```bash
# 单独回放你已录制的视频，并在画面左上角显示 MODE/ACTION 识别结果
roslaunch handcontrol body_pose_only.launch \
  video_path:=/home/cjq/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4

# 若不想循环播放
roslaunch handcontrol body_pose_only.launch \
  video_path:=/home/cjq/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4 \
  loop_video:=false

# 离线识别完整视频，并在视频同目录生成 *_recognition.md 报告
roslaunch handcontrol body_pose_only.launch \
  video_path:=/home/cjq/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4 \
  show_debug:=false \
  loop_video:=false \
  write_report:=true \
  report_path:=auto
```

调试画面会叠加当前模式与动作：`STANDBY`、`BASE`、`ARM`、`SAFETY`，以及右手摇杆/机械臂动作（如 `FORWARD`、`TURN_LEFT`、`EXTEND`、`GRIP_OPEN`）。体感识别节点同时发布 `/body_pose/debug_image`，可用 `rqt_image_view` 查看。`write_report:=true report_path:=auto` 会把时间节点、动作和控制逻辑保存为与视频同目录的 `*_recognition.md`。

### 5.3 自动避障（cmd_vel 融合激光）

```bash
rosrun course_pkg cmd_vel_scan_avoid.py
```
- 订阅 `/cmd_vel_body`（体感输出）与 `/scan`
- 检测到障碍物时自动减速/停止，发布安全 `/cmd_vel`
- 当前一键体感启动默认绕过该节点，作为后续扩展功能保留；要启用时需让体感节点输出 `/cmd_vel_body`，再由本节点转发到 `/cmd_vel`

### 5.4 键鼠 GUI 控制节点（无相机测试）

```bash
roslaunch handcontrol key_mouse_teleop.launch
```
- 直接发布 `/cmd_vel`、`/arm_controller/command`、`/gripper_controller/command`
- 鼠标速度盘支持连续速度控制；键盘支持离散移动和机械臂/夹爪快捷键
- 支持参数覆盖：`roslaunch handcontrol key_mouse_teleop.launch linear_speed:=0.4 angular_speed:=0.8`

### 5.5 机械臂关节控制流程

键盘/键鼠节点不会直接改 Gazebo 关节角度，而是发布 `trajectory_msgs/JointTrajectory` 到 `/arm_controller/command`。Gazebo 中的 `effort_controllers/JointTrajectoryController` 订阅该话题，根据 `/joint_states` 的实际关节角度做 PID，并通过 `gazebo_ros_control` 输出关节力矩。

当前默认控制链路：

```text
key_mouse_teleop.py
  -> /arm_controller/command
  -> arm_controller (effort JointTrajectoryController)
  -> gazebo_ros_control
  -> Gazebo 机械臂关节力矩
```

仿真启动时会自动发送一次机械臂准备姿态，避免控制器只保持接管瞬间的下垂姿态。URDF/控制器参数修改后必须完整重启 Gazebo，`/robot_description` 不会在运行中热更新。

### 5.6 仿真小车摄像头

TB3 Waffle Pi 机械臂模型自带 Gazebo RGB 摄像头，当前话题为 `/camera/rgb/image_raw`，相机信息为 `/camera/rgb/camera_info`。摄像头安装在底盘前上方并向下俯视，主要覆盖机械臂前方抓取区和地面小物块。

为保证 i3-N305 上的帧率，当前仿真相机使用 320x240、10 Hz；实测 `/camera/rgb/image_raw` 约 9.9 Hz。不要再额外叠加第二个 Gazebo 相机，除非明确需要多视角。

### 5.7 保存地图

```bash
rosrun map_server map_saver -f ~/my_map
```

---

## 6. 体感抓取任务阶段规划

最终目标：用体感识别向机器人下命令，让 TB3 + OpenMANIPULATOR-X 小车抓取地面物块，并把物块放到桌子/收集架上。体感负责“下命令”，底盘和机械臂执行预设动作或状态机，不让人体姿态连续直接映射机械臂每个关节。

### 6.1 当前已完成：第一阶段

第一阶段目标是用体感识别替代键鼠控制底盘，并接通机械臂两个离散预设姿态。

- 已实现 `start_body_pose_control.sh` 一键启动 Gazebo + 体感底盘控制
- 已实现 USB 摄像头模式和录制视频回放模式
- 已实现体感节点到 `/cmd_vel` 的直接底盘控制
- 已实现持续姿势持续控制：保持左手底盘模式，右手空中摇杆持续输出线速度和角速度
- 已实现安全停止：双手合十或交叉在胸前时，`teleop_mapper` 立即发布零速度，不经过平滑减速
- 已实现模式切换安全门：`BASE` 和 `ARM` 之间切换必须先经过 `SAFETY`
- 已暂时关闭避障融合，体感直接控制 `/cmd_vel`
- 已实现左手握拳进入机械臂模式，右手上/下离散触发准备姿态和前伸抓取姿态
- 已实现机械臂模式下底盘立即归零，防止机械臂动作时车体继续移动
- 第一阶段不让体感连续控制机械臂关节，避免误动作导致机械臂穿模、翻车或抓取失败

### 6.2 第二阶段：体感命令触发预设动作

第二阶段目标是让体感只负责发离散命令，机械臂执行已经验证过的预设姿态。

建议动作划分：

- 左手张开举在肩旁：底盘模式，右手控制小车接近物块
- 双手合十或交叉胸前：立即停止底盘
- 右手上移：机械臂进入准备姿态，夹爪末端高于车体
- 右手下移：机械臂进入前伸抓取姿态，夹爪伸到车体外并低于物块半高
- 指定体感命令 C：夹爪闭合
- 指定体感命令 D：机械臂抬起并收回
- 指定体感命令 E：放置姿态，夹爪打开

实现方式应优先复用现有 `/arm_controller/command` 和 `/gripper_controller/command`，不要直接改 Gazebo 关节角度。第二阶段需要新增一个“体感命令到预设动作”的状态节点，避免把连续姿态识别直接接到机械臂关节。

### 6.3 第三阶段：半自动抓取状态机

第三阶段目标是把“接近物块、抓取、抬起、移动到桌前、放置”做成明确状态机，体感只负责开始、暂停、继续、急停。

建议状态：

```text
IDLE
  -> APPROACH_OBJECT
  -> ARM_READY
  -> ARM_GRASP_POSE
  -> GRIP_CLOSE
  -> LIFT_AND_RETRACT
  -> APPROACH_TABLE
  -> PLACE_POSE
  -> GRIP_OPEN
  -> ARM_READY
  -> DONE
```

每个状态都要支持安全中断。进入安全锁定时，底盘速度立即归零，机械臂停止接收新动作；解除安全后只能从明确的恢复命令继续，不能自动继续执行危险动作。

### 6.4 第四阶段：感知、导航与避障扩展

第四阶段目标是提高成功率和自动化程度。

- 使用仿真相机 `/camera/rgb/image_raw` 或后续深度/标定信息识别物块位置
- 根据物块相对位置自动微调底盘或机械臂末端
- 恢复并验证 `cmd_vel_scan_avoid.py` 避障融合，避免接近桌子或物块时撞击
- 根据桌子/收集架位置做简单导航或半自动对准
- 如安装 MoveIt，可评估用 MoveIt 做机械臂规划；当前默认不依赖 MoveIt

### 6.5 尚未解决的进阶问题

- 物块识别：当前还没有自动识别“哪个物块要抓”，也没有估计物块精确位姿
- 夹爪对准：当前抓取依赖人工驾驶小车靠近，缺少视觉闭环微调
- 抓取成功判断：当前没有力反馈或视觉确认判断物块是否真的被夹住
- 放置成功判断：当前没有检测物块是否成功落到桌面/收集架
- 机械臂碰撞约束：当前靠预设姿态避免穿模，还没有完整自碰撞/环境碰撞检查
- 车体稳定性：已加前后无动力支撑轮降低前倾，但机械臂伸出过远、速度过快时仍可能晃动
- 体感误识别：光照、遮挡、人体出画、左右手混淆都可能导致误命令，需要继续用视频回放测试阈值
- 命令去抖：模式切换已有稳定帧过滤，但抓取/释放这类一次性动作还需要额外确认机制
- 避障功能：当前体感一键启动默认绕过避障，后续要重新接入 `/cmd_vel_body -> cmd_vel_scan_avoid.py -> /cmd_vel`
- 多节点抢占 `/cmd_vel`：进入体感模式时应确保键鼠、键盘、避障或其他遥控节点没有同时发布 `/cmd_vel`

---

## 7. 常见问题

| 问题 | 解决方法 |
|------|----------|
| `catkin_make` 报 moveit 相关依赖错误 | `turtlebot3_manipulation` 中部分子包已加 `CATKIN_IGNORE`，不影响编译；如需 MoveIt，请 `sudo apt install ros-noetic-moveit-*` 后移除对应 `CATKIN_IGNORE` |
| MediaPipe 安装失败 | 固定版本安装：`/usr/bin/python3 -m pip install mediapipe==0.10.5 --user` |
| Gazebo 模型加载缓慢或失败 | 首次运行需下载模型，或手动拷贝 `course_pkg/gazebo_models/` 到 `~/.gazebo/models/` |
| 摄像头被占用 | 使用 `list_usb_cameras.py` 查看设备号，启动时指定 `camera_index:=1`（或 `-1` 自动探测） |
| 小车原地打滑/抖动 | 降低 `max_linear_speed` / `max_angular_speed`，或调整 `linear_smooth` / `angular_smooth` |
| Gazebo 只有几 FPS 或崩溃 | 键盘/键鼠仿真默认使用 `simple_course.world`；不要默认加载 `pose_course.world` 复杂家居场景，必要时降低窗口分辨率或关闭 Gazebo GUI |
| `Unable to start server[bind: Address already in use]` | 已有 `gzserver` 残留占用 Gazebo 端口；先 `ps -ef | grep gzserver` 查看，确认是旧仿真后结束该进程再启动 |
| 机械臂按 1/2 完全不动 | 确认已安装 `ros-noetic-joint-trajectory-controller` 和 `ros-noetic-effort-controllers`；URDF/控制器修改后必须**重启 Gazebo** |
| 机械臂发软、下垂、像没力 | 检查 `/robot_description` 中 `joint1~joint4` 的 `limit effort`，缩放后应约为 `13.59` 而不是 `1`；若仍为 `1`，完整退出 Gazebo 后重启 |
| 抓住物块后滑落 | 物块质量、物块摩擦、夹爪摩擦、夹爪 effort 和 `gripper_controller` PID 都会影响抓取稳定性；这些参数已调高/调轻，修改后必须完整重启 Gazebo 才会生效 |
| 两爪之间有红色方块 | 已修复：删除 `end_effector_link` 的 visual 模型，重启 Gazebo 生效 |
| 底盘点一下没变化 | 检查是否有多个节点同时发布 `/cmd_vel`（如 `cmd_vel_scan_avoid`），使用 `demo_tb3_keyboard_control.launch` 可避免此问题 |

---

## 8. 目录结构速览

```
catkin_ws/
├── build/          # catkin_make 编译产物
├── devel/          # 环境脚本、库、生成的消息头文件
├── src/
│   ├── course_pkg/                     # 自定义导航/仿真/体感集成
│   │   └── launch/
│   │       └── demo_tb3_keyboard_control.launch   # 键鼠/体感共享 Gazebo 仿真
│   ├── handcontrol/                    # MediaPipe 体感遥控
│   ├── handcontrol_sim/                # 体感仿真辅助
│   ├── turtlebot3/                     # TB3 官方包
│   ├── turtlebot3_manipulation/        # TB3 + 机械臂
│   ├── turtlebot3_manipulation_simulations/  # 机械臂仿真
│   ├── turtlebot3_msgs/                # 消息定义
│   └── Gazebo-Differential-Drive/      # 差速轮演示
├── start_key_mouse_control.sh          # 键鼠 GUI 一键启动脚本
├── start_body_pose_control.sh          # 体感底盘控制一键启动脚本
└── gazebo_models_worlds_collection-master/   # 额外模型与世界
```

---

## 9. 下一步扩展

- **第二阶段体感命令**: 新增体感命令状态节点，用离散姿势触发机械臂准备、前伸抓取、闭合、抬起、放置、打开等预设动作。
- **第三阶段抓取状态机**: 把抓取和放置整理为可暂停、可恢复、可急停的状态机。
- **视觉辅助抓取**: 使用仿真 RGB 相机识别物块位置，先做简单颜色/轮廓检测，再考虑更复杂的位姿估计。
- **避障恢复**: 当前体感底盘控制暂时绕过避障；后续恢复 `cmd_vel_scan_avoid.py`，让体感输出 `/cmd_vel_body`，避障节点最终发布 `/cmd_vel`。
- **导航**: 使用 `demo1.launch` 建图后，配合 `turtlebot3_navigation` 做自主导航 (`amcl` + `move_base`)。
- **机械臂规划**: 如安装 MoveIt，可启用 `turtlebot3_manipulation_moveit_config` 评估规划抓取；当前默认路线仍是预设姿态和状态机。
- **控制界面扩展**: 目前已有键盘、键鼠 GUI、体感三种入口；后续可扩展为 rqt 面板，但当前不需要手柄。

---

