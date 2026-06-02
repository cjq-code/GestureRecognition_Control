# HandControl - ROS1 体感遥控移动底盘

> **填补开源空白**: 第一个 ROS1 + Gazebo + MediaPipe + 普通摄像头的体感遥控差速底盘项目

## 项目概述

使用**普通摄像头** + **MediaPipe Holistic 人体/手势识别**，按工作空间根 README 的体感方案控制 Gazebo 仿真中的差速底盘、机械臂和夹爪。

### 系统架构

```
摄像头 (/dev/videoX)
    ↓ sensor_msgs/Image
[body_pose_estimation.py]  —— MediaPipe Holistic 提取身体和双手关键点
    - 识别左手模式、安全锁、右手摇杆/机械臂动作
    - 发布 /body_pose (自定义消息)
    ↓
[teleop_mapper.py]  —— 状态机 + 速度映射 + 平滑滤波
    - STANDBY(待机) / BASE(底盘) / ARM(机械臂) / SAFETY(安全锁定)
    - 右手摇杆→速度映射
    - 指数移动平均滤波
    - 发布 /cmd_vel (geometry_msgs/Twist)
    ↓
[Gazebo仿真]  —— TurtleBot3 差速底盘
    - 接收 /cmd_vel
    - 执行运动
```

### 控制方式

| 左手动作 | 当前模式 | 右手动作 | 控制结果 |
|----------|----------|----------|----------|
| 左手放下 | STANDBY | 任意 | 底盘停止，机械臂保持当前位置 |
| 左手张开并举在肩旁 | BASE | 右手作为空中摇杆 | 控制底盘前后/转向 |
| 左手握拳并举在肩旁 | ARM | 右手位置 + 手势 | 控制机械臂与夹爪 |
| 双手合十或交叉在胸前 | SAFETY | 任意 | 立即停车，暂停体感控制 |

调试画面会叠加 `MODE` 与 `ACTION`，并发布 `/body_pose/debug_image`。

## 环境要求

### 系统
- **Ubuntu 20.04** (Focal Fossa)
- **ROS1 Noetic**
- **Python 3.8+**

### ROS1 依赖 (apt安装)
```bash
# 基础ROS
sudo apt update
sudo apt install ros-noetic-desktop-full

# 本项目依赖
sudo apt install ros-noetic-cv-bridge ros-noetic-usb-cam ros-noetic-image-transport

# Gazebo仿真 (TurtleBot3)
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-teleop
```

### Python 依赖 (pip安装)
```bash
cd ~/catkin_ws/src/handcontrol
pip3 install -r requirements.txt
```

> **注意**: mediapipe 需要 Python 3.7+，部分系统可能需要 `pip3 install --upgrade pip` 后再安装

## 安装步骤

### 1. 创建工作空间
```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆或复制本项目
git clone <本项目仓库> handcontrol
# 或手动复制 handcontrol/ 文件夹到 ~/catkin_ws/src/
```

### 2. 编译
```bash
cd ~/catkin_ws
catkin_make
# 或 catkin build

# 添加环境变量 (添加到 ~/.bashrc 可永久生效)
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 3. 验证安装
```bash
# 检查包是否可见
rospack find handcontrol
# 应输出: /home/用户名/catkin_ws/src/handcontrol
```

## 运行方法

### 方法A: 4个终端手动启动 (推荐开发调试)

**终端1** - ROS核心
```bash
roscore
```

**终端2** - Gazebo仿真
```bash
# 设置机器人模型
export TURTLEBOT3_MODEL=burger  # 或 waffle, waffle_pi
# 启动Gazebo
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

**终端3** - 体感识别节点
```bash
source ~/catkin_ws/devel/setup.bash
rosrun handcontrol body_pose_estimation.py
# 或带参数: rosrun handcontrol body_pose_estimation.py _camera_index:=1 _show_debug:=true
```

**终端4** - 控制映射节点
```bash
source ~/catkin_ws/devel/setup.bash
rosrun handcontrol teleop_mapper.py
# 或带参数: rosrun handcontrol teleop_mapper.py _max_linear_speed:=0.3
```

### 方法B: Launch一键启动 (推荐日常使用)

```bash
# 先启动Gazebo (终端1)
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 再启动体感控制 (终端2)
source ~/catkin_ws/devel/setup.bash
roslaunch handcontrol handcontrol.launch

# 可选参数
roslaunch handcontrol handcontrol.launch camera_index:=1 max_linear_speed:=0.3 show_debug:=true
```

### 方法C: 单独调试节点

**仅调试姿态识别** (不需要Gazebo)
```bash
roslaunch handcontrol body_pose_only.launch camera_index:=0
# 查看话题: rostopic echo /body_pose
# 查看图像: rqt_image_view 选择 /body_pose/debug_image

# 回放录制的视频做识别测试
roslaunch handcontrol body_pose_only.launch video_path:=~/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4

# 离线分析完整视频，并在视频同目录生成 *_recognition.md
roslaunch handcontrol body_pose_only.launch \
  video_path:=~/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4 \
  show_debug:=false loop_video:=false write_report:=true report_path:=auto
```

## 话题列表

| 话题名 | 类型 | 方向 | 说明 |
|--------|------|------|------|
| `/camera/image_raw` | sensor_msgs/Image | 订阅 | 摄像头原始图像 (可选) |
| `/body_pose` | handcontrol/BodyPose | 发布 | 姿态识别结果 |
| `/body_pose/debug_image` | sensor_msgs/Image | 发布 | 调试图像 (含骨骼和角度) |
| `/cmd_vel` | geometry_msgs/Twist | 发布 | 底盘速度命令 |

## 参数配置

### body_pose_estimation.py

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `camera_index` | 0 | 摄像头设备索引 |
| `video_path` | 空 | 指定 mp4/avi 文件时用视频替代摄像头 |
| `loop_video` | true | 视频结束后循环播放 |
| `mirror_image` | false | 水平翻转输入画面 |
| `write_report` | false | 视频处理结束时写出识别报告 |
| `report_path` | 空 | 报告路径；`auto` 表示保存到视频同目录 `*_recognition.md` |
| `report_sample_interval` | 0.0 | 报告采样间隔秒数，0 表示每帧记录 |
| `use_ros_topic` | false | 使用ROS图像话题代替直连摄像头 |
| `image_topic` | /camera/image_raw | ROS图像话题名 |
| `show_debug` | true | 显示调试图像窗口 |
| `min_detection_confidence` | 0.7 | MediaPipe检测置信度阈值 |
| `min_tracking_confidence` | 0.5 | MediaPipe追踪置信度阈值 |

### teleop_mapper.py

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `max_linear_speed` | 0.5 | 最大线速度 (m/s) |
| `max_angular_speed` | 2.0 | 最大角速度 (rad/s) |
| `linear_smooth_factor` | 0.3 | 线速度平滑系数 (越小越平滑) |
| `angular_smooth_factor` | 0.3 | 角速度平滑系数 |
| `angle_deadzone` | 10.0 | 角度死区 (度) |
| `control_sensitivity` | 1.0 | 控制灵敏度 (>1更灵敏) |
| `wheel_base` | 0.287 | 轮距 (TurtleBot3 Burger) |

## 故障排除

### 摄像头无法打开
```bash
# 列出可用摄像头
v4l2-ctl --list-devices

# 测试摄像头
ffplay /dev/video0

# 检查权限
ls -l /dev/video*
sudo chmod 666 /dev/video0  # 临时解决方案
```

### MediaPipe安装失败
```bash
# 升级pip
pip3 install --upgrade pip setuptools wheel

# 单独安装
pip3 install mediapipe --no-cache-dir

# 如仍失败，尝试指定版本
pip3 install mediapipe==0.10.8
```

### 编译错误 (msg相关)
```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
source devel/setup.bash
```

### Gazebo中机器人不动
```bash
# 检查 /cmd_vel 是否有数据
rostopic echo /cmd_vel

# 检查话题连通性
rostopic info /cmd_vel

# 手动测试键盘控制
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### 速度响应慢/抖动
```bash
# 降低平滑系数 (响应更快)
rosrun handcontrol teleop_mapper.py _linear_smooth_factor:=0.5 _angular_smooth_factor:=0.5

# 或降低死区 (更灵敏)
rosrun handcontrol teleop_mapper.py _angle_deadzone:=5.0
```

## 项目文件结构

```
handcontrol/
├── CMakeLists.txt              # ROS编译配置
├── package.xml                 # 包依赖配置
├── requirements.txt            # Python依赖
├── README.md                   # 本文件
├── msg/
│   └── BodyPose.msg            # 自定义姿态消息
	├── launch/
	│   ├── handcontrol.launch      # 完整启动
	│   ├── body_pose_only.launch   # 仅姿态识别
	│   └── key_mouse_teleop.launch # 键鼠 GUI 控制
├── config/
│   └── (参数配置文件)
└── scripts/
	├── body_pose_estimation.py # 体感识别节点
	├── teleop_mapper.py        # 底盘速度映射节点
	├── manip_teleop_bridge.py  # 体感机械臂离散动作桥接
	└── key_mouse_teleop.py     # 键鼠 GUI 控制节点
```

## 技术亮点

1. **状态机设计**: IDLE/CONTROL/EMERGENCY三状态，安全可靠
2. **平滑滤波**: 指数移动平均 + 渐变限制，消除抖动
3. **速度映射**: S曲线映射，小角度精细控制，大角度快速响应
4. **双臂差速控制**: 符合人体直觉 (像飞机翅膀)
5. **可调参数**: 全部通过ROS参数服务器配置

## 许可证

MIT License - 欢迎Star和Fork!

---

> **实验目的**: 本项目为课程实验设计，展示了如何将MediaPipe视觉感知与ROS机器人控制结合。
