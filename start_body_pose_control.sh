#!/bin/bash
# Start TB3 + OpenMANIPULATOR-X simulation and body-pose base control.

set -e

echo "=========================================="
echo "  启动 TurtleBot3 + OpenMANIPULATOR-X"
echo "  体感底盘控制仿真环境"
echo "=========================================="

source /opt/ros/noetic/setup.bash
source /home/cjq/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
GAZEBO_GUI="${GAZEBO_GUI:-true}"
CAMERA_INDEX="${CAMERA_INDEX:-0}"
SHOW_DEBUG="${SHOW_DEBUG:-true}"
DEFAULT_VIDEO_PATH="/home/cjq/catkin_ws/pose_test_videos/body_pose_test_20260531_024131.mp4"
HANDCONTROL_ARGS=(
    show_debug:="$SHOW_DEBUG"
    max_linear_speed:=0.15
    max_angular_speed:=0.18
    wheel_base:=0.551
    cmd_vel_out_topic:=/cmd_vel
    use_manip_bridge:=true
)

if [ "$CAMERA_INDEX" = "1" ]; then
    VIDEO_PATH="${VIDEO_PATH:-$DEFAULT_VIDEO_PATH}"
    if [ ! -f "$VIDEO_PATH" ]; then
        echo "[ERROR] 录制视频不存在: $VIDEO_PATH"
        exit 1
    fi
    INPUT_LABEL="录制视频: $VIDEO_PATH"
    HANDCONTROL_ARGS+=(camera_index:=0 video_path:="$VIDEO_PATH" loop_video:=true)
else
    INPUT_LABEL="外接 USB 摄像头 index=$CAMERA_INDEX"
    HANDCONTROL_ARGS+=(camera_index:="$CAMERA_INDEX" video_path:= loop_video:=false)
fi

echo ""
echo "[1/2] 启动 Gazebo + 机械臂控制器 ..."
echo "      请等待 Gazebo 完全加载（约 10-20 秒）"
echo ""

roslaunch course_pkg demo_tb3_keyboard_control.launch gui:="$GAZEBO_GUI" &
LAUNCH_PID=$!

cleanup() {
    if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
        kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
        wait "$LAUNCH_PID" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

sleep 12

if ! rosnode list 2>/dev/null | grep -qx "/gazebo"; then
    echo ""
    echo "[ERROR] Gazebo 未正常运行，停止启动体感控制。"
    echo "        请查看上方 Gazebo 日志，或检查是否有旧 gzserver 残留。"
    exit 1
fi

echo ""
echo "[2/2] 启动体感识别 + 底盘控制 ..."
echo "      输入源: $INPUT_LABEL"
echo "      左手张开举在肩旁进入底盘模式，右手作为空中摇杆"
echo "      双手合十或交叉在胸前会立即发布 /cmd_vel=0"
echo "      roslaunch 参数: ${HANDCONTROL_ARGS[*]}"
echo ""

roslaunch handcontrol handcontrol.launch "${HANDCONTROL_ARGS[@]}"
