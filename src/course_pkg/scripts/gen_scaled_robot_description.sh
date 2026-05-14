#!/usr/bin/env bash
# Generate scaled TurtleBot3 + OpenMANIPULATOR URDF for course_pkg launch.
set -euo pipefail
SCALE="${1:-1.92}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XACRO="$(rospack find turtlebot3_manipulation_description)/urdf/turtlebot3_manipulation_robot.urdf.xacro"
rosrun xacro xacro --inorder "$XACRO" | python3 "$SCRIPT_DIR/scale_urdf_for_course.py" "$SCALE"
