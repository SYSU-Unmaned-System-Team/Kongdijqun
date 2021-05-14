std_msgs/Header header

## N号飞机执行任务标志位，1代表执行，0代表不执行任务
##   1号飞机 2号飞机 3号飞机 4号飞机       Flag_id
##      是              否           是            是               0b1101
##      否              否           是            是               0b1100
uint8 Flag_id

float32[3] goal_uav1
float32[3] goal_uav2
float32[3] goal_uav3
float32[3] goal_uav4

## 控制命令的模式 
uint8 Command_uav
# enum Command_uav 控制模式枚举
uint8 Start=0
uint8 Return=1
uint8 Land=2
uint8 Normal=3


uint8 Command_ugv

uint8 detection_flag
# enum detection_flag 控制模式枚举
uint8 no_one_detected=0
uint8 detected=1


## 目标位置-全局坐标系-(x,y,z)
float32[3] enu_position


