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
    echo "[ERROR] Gazebo 未正常运行，停止启动键鼠控制。"
    echo "        请查看上方 Gazebo 日志，或检查是否有旧 gzserver 残留。"
    exit 1
fi

echo ""
echo "[2/2] 启动键鼠控制 GUI ..."
echo ""

roslaunch handcontrol key_mouse_teleop.launch
