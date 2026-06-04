#!/bin/bash
# Start TB3 + OpenMANIPULATOR-X simulation and the key/mouse teleop GUI.

set -e

echo "=========================================="
echo "  启动 TurtleBot3 + OpenMANIPULATOR-X"
echo "  键鼠控制仿真环境"
echo "=========================================="

source /opt/ros/noetic/setup.bash
source /home/cjq/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
GAZEBO_GUI="${GAZEBO_GUI:-true}"
AUTO_VISION="${AUTO_VISION:-true}"
VISION_TARGET_COLOR="${VISION_TARGET_COLOR:-red}"

echo ""
echo "[1/3] 启动 Gazebo + 机械臂控制器 ..."
echo "      请等待 Gazebo 完全加载（约 10-20 秒）"
echo ""

roslaunch course_pkg demo_tb3_keyboard_control.launch gui:="$GAZEBO_GUI" &
LAUNCH_PID=$!
VISION_PID=""

cleanup() {
    if [ -n "$VISION_PID" ] && kill -0 "$VISION_PID" >/dev/null 2>&1; then
        kill -INT "$VISION_PID" >/dev/null 2>&1 || true
        wait "$VISION_PID" >/dev/null 2>&1 || true
    fi
    if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
        kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
        wait "$LAUNCH_PID" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

sleep 12

if ! rosnode list 2>/dev/null | grep -qx "/gazebo"; then
    echo ""
    echo "[ERROR] Gazebo 未正常运行，停止启动键鼠控制。"
    echo "        请查看上方 Gazebo 日志，或检查是否有旧 gzserver 残留。"
    exit 1
fi

echo ""
if [ "$AUTO_VISION" = "true" ]; then
    echo "[2/3] 启动视觉自动抓取 ..."
    echo "      看到 ${VISION_TARGET_COLOR} 目标后自动接管，抓取完成后恢复键鼠控制"
    echo ""
    roslaunch handcontrol vision_align_demo.launch target_color:="$VISION_TARGET_COLOR" &
    VISION_PID=$!
    sleep 2
else
    echo "[2/3] 跳过视觉自动抓取（AUTO_VISION=false）"
    echo ""
fi

echo "[3/3] 启动键鼠控制 GUI ..."
echo ""

roslaunch handcontrol key_mouse_teleop.launch
