# 人体姿态消息 - MediaPipe Pose 33关键点精简版
# 双臂状态用于控制差速底盘

Header header

# 姿态状态: 0=IDLE待机, 1=CONTROL控制中, 2=EMERGENCY紧急停止
int8 state

# 左臂角度 (度): -90(后) ~ +90(前), 0=水平
float32 left_arm_angle

# 右臂角度 (度): -90(后) ~ +90(前), 0=水平
float32 right_arm_angle

# 左臂状态: 0=放下, 1=抬起
int8 left_arm_up

# 右臂状态: 0=放下, 1=抬起
int8 right_arm_up

# 人体中心位置 (归一化 0.0~1.0, 0.5为中心)
float32 body_center_x
float32 body_center_y

# 检测到的人数
int32 num_people

# --- course_pkg 体感课程模式（Holistic：手势 + 全身）---
# 是否使用课程映射（teleop_mapper 中 ~use_course_mapping 为 true 时读取下列字段）
int8 use_course_fields

# 底盘运动已解锁：左手稳定比出「2」后置 1，直到双手合十/并拢取消
int8 course_base_motion_enabled

# 当前帧检测到左手「2」手势（未做门闩，用于调试）
int8 left_hand_two_detected

# 双手合十/并拢（手掌靠近）——取消底盘运动
int8 palms_together

# 右手掌张开程度 0=握拳/并拢 1=完全张开（映射夹爪）
float32 right_palm_open_score

# 双臂肘部平均弯曲量（度）：0 近似伸直，越大越弯（映射机械臂关节）
float32 avg_elbow_bend_deg

# 腿部「向前」驱动意图 0~1（膝关节屈曲等启发式）
float32 leg_forward_norm

# 腿部侧向意图 -1~1，正值表示更偏向右侧迈步（映射右转）
float32 leg_lateral_norm

# --- README 当前体感交互方案（左手模式 + 右手连续控制）---
# 0=待机, 1=底盘模式, 2=机械臂模式, 3=安全锁定
int8 control_mode

# 便于调试画面/rostopic echo 直接查看当前识别结果
string mode_label
string action_label

# 左/右手手势分类
int8 left_palm_open
int8 left_fist
int8 right_palm_open
int8 right_fist

# 右手相对右肩附近零点的归一化控制量：x 右正，y 上正
float32 right_hand_x_norm
float32 right_hand_y_norm

# 右臂伸展意图：-1 收回，+1 前伸（单目摄像头下由肘角和肩腕距离估计）
float32 right_hand_depth_norm
