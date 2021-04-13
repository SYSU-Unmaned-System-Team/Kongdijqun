### CASE 2 协同搜索



**global_planner.cpp程序主逻辑**：

- 设定好第x架无人机的飞行探测区域，高度
- 人为切换offboard及解锁（实飞），仿真中自动切换并解锁
- 程序检测到offboard模式后，从起飞点起飞，抵达预定高度后，进入WAIT_GOAL模式
- 状态机switch：
- INIT：初始化
  - 仿真模式：等待输入，进入TAKEOFF
  - 实飞模式：等待遥控器切换offboard指令及解锁、等待地面站发送takeoff指令，进入TAKEOFF
- TAKEOFF：发送起飞指令，进入WAIT_GOAL
  - 设定返航点
- WAIT_GOAL：等待目标点或设定目标点
  - 依次执行目标点，进入PLANNING
  - 若未设置目标点，则等待手动输入目标点，进入PLANNING
  - 执行完最后一个航点，进入RETURN
- PLANNING：执行A星算法
  - 增加逻辑：即使某一个目标点被占据，则直接飞往下一个目标点（即返回WAIT_GOAL）
  - 寻找到路径，设定path_ok=true，进入PATH_TRACKING
  - 未寻找到路径，设定path_ok=false，返回WAIT_GOAL
- PATH_TRACKING：定时返回PLANNING重新规划
  - path_ok=true：执行航点，若抵达最后一个航点，返回WAIT_GOAL
- OBJECT_TRACKING：
  - 静态目标：直接飞至目标上方
  - 动态目标：追踪目标
- RETURN_PLANNING：设定目标点为返航点，执行规划
  - 找不到路径，进入LAND 
- RETURN：返航
  - 接近返航点，进入LAND
- LAND：原地降落
- **目标检测回调**：订阅目标检测结果
  - if (detected_by_others) 代表其他飞机已检测到，直接return
  - if (detected_by_me) 代表本机检测，此时存储检测值用于目标追踪控制
    - 连续N帧检测不到，则detected设置为false
  - 上述情况都不满足，初始检测：连续检测到N帧后（同时要判断该目标是否在他的区域内）
    - 设定detected_by_me，即本机找到目标
    - 设定状态机进入OBJECT_TRACKING
    - 发布检测结果至地面站
  - 相机坐标系：从相机往前看，物体在相机右方x为正，下方y为正，前方z为正
- **地面站指令回调**：订阅来自地面站的广播消息
  - Command_UAV: 起飞、return、降落
  - 检测结果
    - detected
    - no_one_detected
    - enu_position
  - if (obeject_status == other_get) 直接执行返航目标点，进入RETURN
- **追踪路径定时回调**：
  - path_ok==false，返回
  - path_ok==true，追踪路径
  - 抵达最后一个目标点，进入WAIT_GOAL
- **目标追踪定时回调**：
  - if (object_status!=2 ||不是OBJECT_TRACKING状态机)，返回
  - if (detected==false),无人机悬停
  - xy速度+z定高追踪
  - 待思考：如何降落并结束任务？

注意事项：

- 尽量不要让两个飞机可以同时看到marker





小车逻辑：

接收到地面站指令后，计算最近的小车，然后避障前往目标点。