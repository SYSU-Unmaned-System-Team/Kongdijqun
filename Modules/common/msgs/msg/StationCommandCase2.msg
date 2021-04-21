std_msgs/Header header

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


