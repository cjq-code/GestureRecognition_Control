# 项目展示指令速查

本文档专门整理汇报演示时常用命令，覆盖车体展示、场景展示、体感识别、键鼠控制、体感识别控制。

所有命令默认在工作空间根目录执行：

```bash
cd ~/catkin_ws
source devel/setup.bash
```

## 0. 录制与回放 Demo 视频

### 0.1 录制体感识别测试视频

用途：提前录一段人体动作视频，汇报时可以不用现场摄像头，直接回放视频做 MediaPipe 识别和体感控制演示。

默认录制 30 秒，保存到：

```text
~/catkin_ws/pose_test_videos/
```

命令：

```bash
rosrun handcontrol record_pose_test_video.py
```

自动选择第一个可用摄像头，录制 60 秒：

```bash
rosrun handcontrol record_pose_test_video.py --camera-index -1 --duration 60
```

无图形界面录制：

```bash
rosrun handcontrol record_pose_test_video.py --camera-index -1 --duration 60 --no-preview
```

说明：

- 录制完成后，终端会打印实际保存的视频文件路径
- 默认文件夹是 `/home/cjq/catkin_ws/pose_test_videos/`
- 视频文件名一般类似 `body_pose_test_20260531_024131.mp4`
- 后面所有 `video_path:=...` 或 `VIDEO_PATH=...` 都要改成你实际录出来的视频路径

### 0.2 统一设置视频路径

为了避免每条命令都改一遍，可以先在终端设置一个变量：

```bash
VIDEO_FILE=/home/cjq/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4
```

如果你录制了新视频，只需要把上面这一行改成新文件名，例如：

```bash
VIDEO_FILE=/home/cjq/catkin_ws/pose_test_videos/body_pose_test_20260603_153000.mp4
```

后面使用视频回放时，优先使用 `$VIDEO_FILE`。

如果忘了最新录制的视频文件名，可以查看目录：

```bash
ls -lh ~/catkin_ws/pose_test_videos/
```

也可以自动取最新的 mp4：

```bash
VIDEO_FILE=$(ls -t ~/catkin_ws/pose_test_videos/*.mp4 | head -n 1)
echo $VIDEO_FILE
```

## 1. 车体展示 TurtleBot3 + OpenManipulator 车体模型

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_description turtlebot3_manipulation_view.launch use_gui:=true
```

说明：
- 打开 RViz
- `use_gui:=true` 会打开关节滑块，方便手动拖动机械臂关节



## 2. 场景展示

### 2.1 展示默认轻量场景 + TurtleBot3 机械臂

用途：展示当前主项目默认仿真场景，包含 TurtleBot3 + OpenManipulator 和轻量 Gazebo 场景。

```bash
roslaunch course_pkg demo_tb3_keyboard_control.launch gui:=true
```

说明：

- 默认世界：`src/course_pkg/worlds/simple_course.world`
- 默认模型路径：`src/course_pkg/gazebo_models`
- 这是键鼠控制和体感控制共用的 Gazebo 仿真入口

### 2.2 只展示 Gazebo 场景


```bash
export GAZEBO_MODEL_PATH=~/catkin_ws/src/course_pkg/gazebo_models:$GAZEBO_MODEL_PATH
roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find course_pkg)/worlds/simple_course.world
```


## 3. 体感识别展示

### 3.1 MediaPipe 手部动作测试程序

用途：只测试 MediaPipe 识别，不控制小车，不发布 `/cmd_vel`。

识别内容：

- 左手是否张开
- 右手是否张开
- 左手相对左肩原点的方位
- 右手相对右肩原点的方位

使用摄像头：

```bash
roslaunch handcontrol hand_action_demo.launch camera_index:=0
```

使用录制视频：

```bash
roslaunch handcontrol hand_action_demo.launch \
  video_path:=$VIDEO_FILE \
  loop_video:=false
```

查看识别 JSON：

```bash
rostopic echo /hand_action_demo/status
```

查看调试图像：

```bash
rqt_image_view /hand_action_demo/debug_image
```

方位标签：

```text
CENTER, UP, DOWN, LEFT, RIGHT, UP_LEFT, UP_RIGHT, DOWN_LEFT, DOWN_RIGHT
```

### 3.2 完整体感模式识别测试

用途：测试正式体感识别节点，只看 `STANDBY`、`BASE`、`ARM`、`SAFETY` 和 `ACTION`，不启动小车仿真。

使用摄像头：

```bash
roslaunch handcontrol body_pose_only.launch camera_index:=0 show_debug:=true
```

使用录制视频：

```bash
roslaunch handcontrol body_pose_only.launch \
  video_path:=$VIDEO_FILE \
  loop_video:=false \
  show_debug:=true
```

查看识别话题：

```bash
rostopic echo /body_pose/mode_label
rostopic echo /body_pose/action_label
rostopic echo /body_pose/right_hand_x_norm
rostopic echo /body_pose/right_hand_y_norm
```

查看调试图像：

```bash
rqt_image_view /body_pose/debug_image
```

## 4. 键鼠控制展示

### 4.1 一键启动 Gazebo + 键鼠控制 GUI


```bash
./start_key_mouse_control.sh
```

控制说明：

```text
W/A/S/D 或方向键：控制底盘
鼠标速度盘：拖拽控制底盘速度
X 或空格：底盘停止
R/U：急停/解除急停
1/2：机械臂准备姿态/前伸抓取姿态
O/C：夹爪张开/闭合
```

### 4.2 只启动键鼠 GUI

用途：Gazebo 已经在其他终端启动时，只接入键鼠控制节点。

```bash
roslaunch handcontrol key_mouse_teleop.launch
```

## 5. 体感识别控制展示

### 5.1 一键启动 Gazebo + 体感控制

用途：完整演示体感识别控制 TurtleBot3 机械臂仿真。

```bash
./start_body_pose_control.sh
```

使用摄像头：

```bash
CAMERA_INDEX=0 ./start_body_pose_control.sh
```

使用录制视频：

```bash
CAMERA_INDEX=1 VIDEO_PATH=$VIDEO_FILE ./start_body_pose_control.sh
```

这里必须把 `VIDEO_FILE` 或 `VIDEO_PATH` 改成实际视频文件名。`CAMERA_INDEX=1` 表示不打开 USB 摄像头，改用录制视频回放。

### 5.2 体感控制动作规则

```text
左手放下：待机，底盘停止
左手张开并举在肩旁：进入 BASE 底盘模式
左手握拳并举在肩旁：进入 ARM 机械臂模式
双手合十或交叉胸前：进入 SAFETY 安全锁定
```

底盘模式：

```text
右手上移：前进
右手下移：后退
右手左移：左转
右手右移：右转
```

机械臂模式：

```text
右手上移：机械臂准备/抬起姿态
右手下移：机械臂前伸抓取/放下姿态
右手握拳：夹爪闭合
右手张开：夹爪张开
```

注意：

- `BASE` 和 `ARM` 不能直接互切，必须先进入 `SAFETY`
- 左右方向按操作者本人视角定义，不按摄像头画面坐标定义


## 6. 常用排查命令

查看 ROS 节点：

```bash
rosnode list
```

查看底盘速度：

```bash
rostopic echo /cmd_vel
```

查看机械臂关节状态：

```bash
rostopic echo /joint_states
```

查看体感识别状态：

```bash
rostopic echo /body_pose/mode_label
rostopic echo /body_pose/action_label
```
