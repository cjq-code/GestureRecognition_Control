cmake_minimum_required(VERSION 3.0.2)
project(handcontrol)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

## 添加自定义消息
add_message_files(
  FILES
  BodyPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    message_runtime 
    cv_bridge
)

## 安装Python脚本
catkin_install_python(PROGRAMS
  scripts/body_pose_estimation.py
  scripts/teleop_mapper.py
  scripts/manip_teleop_bridge.py
  scripts/list_usb_cameras.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装msg文件
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)