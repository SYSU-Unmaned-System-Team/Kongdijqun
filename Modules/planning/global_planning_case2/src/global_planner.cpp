#include "global_planner.h"

namespace Global_Planning
{

// 初始化函数
void Global_Planner::init(ros::NodeHandle& nh)
{
    // 读取参数
    // 集群数量
    nh.param<int>("swarm_num", swarm_num, 1);
    // 无人机编号 1号无人机则为1
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<string>("uav_name", uav_name, "/uav0");
    // 是否为仿真模式
    nh.param("global_planner/sim_mode", sim_mode, false); 
    // A星算法 重规划频率 
    nh.param("global_planner/replan_time", replan_time, 2.0); 
    // 2D规划时,定高高度
    nh.param("global_planner/fly_height_2D", fly_height_2D, 1.0);  
    // 路径追踪间隔
    nh.param("global_planner/time_per_path", time_per_path, 1.0); 
    // 追踪速度
    nh.param("global_planner/velocity_path_tracking", velocity_path_tracking, 0.5); 
    // 手动给定目标点模式 或 自动目标点模式
    nh.param("global_planner/manual_mode", manual_mode, false);
    // 检测范围
    nh.param("global_planner/detection_range_x_min", detection_range_x(0), 0.0);
    nh.param("global_planner/detection_range_x_max", detection_range_x(1), 0.0);
    nh.param("global_planner/detection_range_y_min", detection_range_y(0), 0.0);
    nh.param("global_planner/detection_range_y_max", detection_range_y(1), 0.0);
            
    if(!manual_mode)
    {
        // 自动目标点模式，读取目标点
        nh.param("global_planner/goal_num", goal_num, 1); 

        for(int i = 0; i < goal_num; i++) 
        {
            nh.param("global_planner/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
            nh.param("global_planner/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
            waypoints_[i][2] = fly_height_2D;

            cout << "goal("<<i<<"): ["<< waypoints_[i][0] << ", "  << waypoints_[i][1]  << ", "  << waypoints_[i][2] << " ]"   <<endl;
        }
        goal_id = 0;
    }else
    {
        // 手动给定目标点模式 手动目标点
        goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/prometheus/planning/goal", 1, &Global_Planner::goal_cb, this);
    }
    
    // 【地面站交互】订阅地面站控制指令（所有从机均使用该指令）
    station_cmd_sub = nh.subscribe<prometheus_msgs::StationCommandCase2>("/case2/command_uav", 1, &Global_Planner::cmd_cb, this);
    // 【地面站交互】将检测结果发送至地面站（所有从机均使用该指令）
    case2_result_pub = nh.advertise<prometheus_msgs::Case2Result>("/case2/uav_result", 10);
    // 【地面站交互】发布提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // 订阅 无人机状态
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10, &Global_Planner::drone_state_cb, this);

    // 目前为aruco码检测，后期更换为YOLO检测
    detection_sub = nh.subscribe<prometheus_msgs::ArucoInfo>(uav_name + "/prometheus/object_detection/aruco_det", 10, &Global_Planner::detection_cb, this);

    // 选择地图更新方式：　0代表全局点云，1代表局部点云，2代表激光雷达scan数据
    nh.param("global_planner/map_input", map_input, 0); 
    // 根据map_input选择地图更新方式
    if(map_input == 0)
    {
        Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(uav_name + "/prometheus/global_planning/global_pcl", 1, &Global_Planner::Gpointcloud_cb, this);
    }else if(map_input == 1)
    {
        Lpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(uav_name + "/prometheus/global_planning/local_pcl", 1, &Global_Planner::Lpointcloud_cb, this);
    }else if(map_input == 2)
    {
        laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>(uav_name + "/prometheus/global_planning/laser_scan", 1, &Global_Planner::laser_cb, this);
    }

    // 发布 路径指令 （发送至swarm_controller.cpp）
    command_pub = nh.advertise<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 10);
    // 发布路径用于显示（rviz显示）
    path_cmd_pub   = nh.advertise<nav_msgs::Path>(uav_name + "/prometheus/global_planning/path_cmd",  10); 
    // 主循环执行定时器
    mainloop_timer = nh.createTimer(ros::Duration(1.0), &Global_Planner::mainloop_cb, this);        
    // 路径追踪定时器
    track_path_timer = nh.createTimer(ros::Duration(time_per_path), &Global_Planner::track_path_cb, this);        
    // 物体追踪定时器
    object_tracking_timer = nh.createTimer(ros::Duration(0.1), &Global_Planner::object_tracking_cb, this);  

    // Astar algorithm
    Astar_ptr.reset(new Astar);
    Astar_ptr->init(nh);
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "A_star init.");

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::INIT;
    odom_ready = false;
    drone_ready = false;
    get_goal = false;
    station_ready = false;
    sensor_ready = false;
    is_new_path = false;
    path_ok = false;

    // 检测相关
    detected_by_myself = false;
    detected_by_others = false;
    num_count_vision_lost = 0;
    object_pos_body << 0.0, 0.0, 0.0; 
    object_pos_enu << 0.0, 0.0, 0.0;
    lost_object = true;

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::SwarmCommand::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.source = NODE_NAME;
    desired_yaw = 0.0;
    command_pub.publish(Command_Now);
}


void Global_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // 2D定高飞行
    goal_pos << msg->pose.position.x, msg->pose.position.y, fly_height_2D;
        
    goal_vel.setZero();

    get_goal = true;

    char message_chars[256];
    sprintf(message_chars, "Get a new goal point: [%f, %f, %f].", goal_pos(0),goal_pos(1),goal_pos(2));
    message = message_chars;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);

    // 当目标点为[99,99]时,无人机原地降落
    if(goal_pos(0) == 99 && goal_pos(1) == 99 )
    {
        path_ok = false;
        get_goal = false;
        exec_state = EXEC_STATE::LAND;
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Get Land CMD.");
    }

}

void Global_Planner::cmd_cb(const prometheus_msgs::StationCommandCase2ConstPtr& msg)
{
    station_cmd = *msg;

    if(station_cmd.Command_uav == prometheus_msgs::StationCommandCase2::Start)
    {
        station_ready = true;
        return;
    }else if(station_cmd.Command_uav == prometheus_msgs::StationCommandCase2::Return)
    {
        exec_state = EXEC_STATE::RETURN;
        message = "Get command from ground station: [ RETURN ]";
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
        return;
    }else if(station_cmd.Command_uav == prometheus_msgs::StationCommandCase2::Land)
    {
        exec_state = EXEC_STATE::LAND;
        message = "Get command from ground station: [ LAND ]";
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
        return;
    }

    if(station_cmd.detection_flag == prometheus_msgs::StationCommandCase2::detected && detected_by_myself == false)
    {
        // 如果地面站说已检测到，但并不是自己检测到的，则就是别人检测到了
        // 本消息只会收到一次
        detected_by_others = true;
        exec_state = EXEC_STATE::RETURN;
        message = "Detected by others，return.";
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
    }
}

void Global_Planner::detection_cb(const prometheus_msgs::ArucoInfoConstPtr &msg)
{
    // 如果其他无人机检测到,则不再处理接收本机的检测结果
    if(detected_by_others)
    {
        return;
    }
    
    if(msg->detected)
    {
        num_count_vision_get++;
        num_count_vision_lost = 0;
    }else
    {
        num_count_vision_lost++;
        num_count_vision_get = 0;
    }

    if(detected_by_myself)
    {
        if(num_count_vision_lost > 10)
        {
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,"can't see target.");
        }
        //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
        // 注意，此处与无人机姿态相关！！
        object_pos_body << - msg->position[1], - msg->position[0], -fly_height_2D;

        Eigen::Vector3d enu_frame;
        enu_frame[0] = object_pos_body[0] * cos(_DroneState.attitude[2]) - object_pos_body[1] * sin(_DroneState.attitude[2]);
        enu_frame[1] = object_pos_body[0] * sin(_DroneState.attitude[2]) + object_pos_body[1] * cos(_DroneState.attitude[2]);
        enu_frame[2] = -fly_height_2D;

        object_pos_enu = enu_frame + start_pos;

    }else
    {
        // 当连续到10帧，且目标在本无人机的任务范围内，则判定本机检测到目标
        if(num_count_vision_get > 10)
        {
            object_pos_body << - msg->position[1], - msg->position[0], -fly_height_2D;

            Eigen::Vector3d enu_frame;
            enu_frame[0] = object_pos_body[0] * cos(_DroneState.attitude[2]) - object_pos_body[1] * sin(_DroneState.attitude[2]);
            enu_frame[1] = object_pos_body[0] * sin(_DroneState.attitude[2]) + object_pos_body[1] * cos(_DroneState.attitude[2]);
            enu_frame[2] = -fly_height_2D;

            object_pos_enu = enu_frame + start_pos;
            
            // 判定目标是否在范围内
            if(object_pos_enu(0)<detection_range_x(1) && object_pos_enu(0)>detection_range_x(0) 
            && object_pos_enu(1)<detection_range_y(1) && object_pos_enu(1)>detection_range_y(0))
            {
                detected_by_myself = true;

                // 停止执行路径追踪
                path_ok = false;    
                // 开始执行物体追踪
                exec_state = EXEC_STATE::OBEJECT_TRACKING;

                prometheus_msgs::Case2Result result;

                result.uav_id = uav_id;

                result.enu_position[0] = object_pos_enu[0];
                result.enu_position[1] = object_pos_enu[1];
                result.enu_position[2] = object_pos_enu[2];

                // 发布
                case2_result_pub.publish(result);
            }else
            {
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,"target not in our zone.");
            }
            
        }
    }

}

void Global_Planner::drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg)
{
    _DroneState = *msg;

    if (_DroneState.connected == true)
    {
        drone_ready = true;
    }else
    {
        drone_ready = false;
    }

    odom_ready = true;

    //无人机里程计，用于建图
    Drone_odom.header = _DroneState.header;
    Drone_odom.child_frame_id = "base_link";

    Drone_odom.pose.pose.position.x = _DroneState.position[0];
    Drone_odom.pose.pose.position.y = _DroneState.position[1];
    Drone_odom.pose.pose.position.z = _DroneState.position[2];
    Drone_odom.pose.pose.orientation = _DroneState.attitude_q;

    Drone_odom.twist.twist.linear.x = _DroneState.velocity[0];
    Drone_odom.twist.twist.linear.y = _DroneState.velocity[1];
    Drone_odom.twist.twist.linear.z = _DroneState.velocity[2];

    // 更新无人机初始位置、速度、加速度，用于规划
    start_pos << msg->position[0], msg->position[1], fly_height_2D;
    start_vel << msg->velocity[0], msg->velocity[1], 0.0;
    start_acc << 0.0, 0.0, 0.0;

    // 计算行走距离
    distance_walked = (uav_pos_last - start_pos).norm();
}

// 根据全局点云更新地图
// 情况：已知全局点云的场景、由SLAM实时获取的全局点云
void Global_Planner::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;

    static int update_num=0;
    update_num++;

    // 此处改为根据循环时间计算的数值
    if(update_num == 10)
    {
        // 对Astar中的地图进行更新
        Astar_ptr->Occupy_map_ptr->map_update_gpcl(msg);
        // 并对地图进行膨胀
        Astar_ptr->Occupy_map_ptr->inflate_point_cloud(); 
        update_num = 0;
    } 
}

// 根据局部点云更新地图
// 情况：RGBD相机、三维激光雷达
void Global_Planner::Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;
    
    if(exec_state != EXEC_STATE::INIT && exec_state != EXEC_STATE::LAND)
    {
        // 若无人机实际高度与定点高度差值超过0.2米，则发出警告，且不更新地图
        if (abs( Drone_odom.pose.pose.position.z - fly_height_2D) > 0.2)
        {
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"UAV is not in desired height.");
        }else
        {
            // 对Astar中的地图进行更新（局部地图+odom）
            Astar_ptr->Occupy_map_ptr->map_update_lpcl(msg, Drone_odom);
            // 并对地图进行膨胀
            Astar_ptr->Occupy_map_ptr->inflate_point_cloud(); 
        }
    }

}

// 根据2维雷达数据更新地图
// 情况：2维激光雷达
void Global_Planner::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;
    
    if(exec_state != EXEC_STATE::INIT && exec_state != EXEC_STATE::LAND)
    {
        // 若无人机实际高度与定点高度差值超过0.2米，则发出警告，且不更新地图
        if (abs( Drone_odom.pose.pose.position.z - fly_height_2D) > 0.2)
        {
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"UAV is not in desired height.");
        }else
        {
            // 对Astar中的地图进行更新（laser+odom）
            Astar_ptr->Occupy_map_ptr->map_update_laser(msg, Drone_odom);
            // 并对地图进行膨胀
            Astar_ptr->Occupy_map_ptr->inflate_point_cloud(); 
        }
    }
}

void Global_Planner::track_path_cb(const ros::TimerEvent& e)
{
    static int track_path_num = 0;
    
    if(!path_ok)
    {
        return;
    }

    is_new_path = false;

    // 抵达终点
    if(cur_id == Num_total_wp - 1)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::SwarmCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Move_mode           = prometheus_msgs::SwarmCommand::XYZ_POS;
        Command_Now.position_ref[0]     = path_cmd.poses[Num_total_wp-1].pose.position.x;
        Command_Now.position_ref[1]     = path_cmd.poses[Num_total_wp-1].pose.position.y;
        Command_Now.position_ref[2]     = path_cmd.poses[Num_total_wp-1].pose.position.z;
        Command_Now.yaw_ref             = desired_yaw;
        command_pub.publish(Command_Now);

        char message_chars[256];
        sprintf(message_chars, "Reach the goal.");
        message = message_chars;
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
        
        // 停止执行
        path_ok = false;

        if(exec_state == EXEC_STATE::RETURN)
        {
            exec_state = EXEC_STATE::LAND;
        }else 
        {
            exec_state = EXEC_STATE::WAIT_GOAL;
        }

        return;
    }

    int i = cur_id;

    cout << "Moving to Waypoint: [ " << cur_id << " / "<< Num_total_wp<< " ] "<<endl;
    cout << "Moving to Waypoint:"   << path_cmd.poses[i].pose.position.x  << " [m] "
                                    << path_cmd.poses[i].pose.position.y  << " [m] "
                                    << path_cmd.poses[i].pose.position.z  << " [m] "<<endl; 
    
    // 控制方式如果是走航点，则需要对无人机进行限速，保证无人机的平滑移动
    // 采用轨迹控制的方式进行追踪，期望速度 = （期望位置 - 当前位置）/预计时间；
    // 此处控制方式待定
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::SwarmCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;

    if(false)
    {
        Command_Now.Move_mode           = prometheus_msgs::SwarmCommand::TRAJECTORY;
        Command_Now.position_ref[0]     = path_cmd.poses[i].pose.position.x;
        Command_Now.position_ref[1]     = path_cmd.poses[i].pose.position.y;
        Command_Now.position_ref[2]     = path_cmd.poses[i].pose.position.z;
        Command_Now.velocity_ref[0]     = path_cmd.poses[i].pose.position.x - _DroneState.position[0];
        Command_Now.velocity_ref[1]     = path_cmd.poses[i].pose.position.y - _DroneState.position[1];
        float error = sqrtf((pow(Command_Now.velocity_ref[0],2) + pow(Command_Now.velocity_ref[1],2)));
        Command_Now.velocity_ref[0]     = Command_Now.velocity_ref[0] / error * velocity_path_tracking;
        Command_Now.velocity_ref[1]     = Command_Now.velocity_ref[1] / error * velocity_path_tracking;
        Command_Now.yaw_ref             = desired_yaw;
        cur_id = cur_id + 1;
    }else if (true)
    {
        Command_Now.Move_mode           = prometheus_msgs::SwarmCommand::XY_VEL_Z_POS;
        Command_Now.velocity_ref[0]     = path_cmd.poses[i].pose.position.x - _DroneState.position[0];
        Command_Now.velocity_ref[1]     = path_cmd.poses[i].pose.position.y - _DroneState.position[1];
        float error = sqrtf((pow(Command_Now.velocity_ref[0],2) + pow(Command_Now.velocity_ref[1],2)));
        Command_Now.velocity_ref[0]     = Command_Now.velocity_ref[0] / error * velocity_path_tracking;
        Command_Now.velocity_ref[1]     = Command_Now.velocity_ref[1] / error * velocity_path_tracking;
        Command_Now.position_ref[2]     = path_cmd.poses[i].pose.position.z;
        Command_Now.yaw_ref             = desired_yaw;  

        error = sqrtf((pow(Command_Now.velocity_ref[0],2) + pow(Command_Now.velocity_ref[1],2)));

        cout << "velocity_ref: [ " << error <<  " ] "<<endl;
        
        track_path_num++;
        // 5 = (resolution/期望速度)/time_per_path(即0.1)
        if(track_path_num % 5 == 0)
        {
            cur_id = cur_id + 1;
        }
    }
    
    command_pub.publish(Command_Now);   
}
 

void Global_Planner::object_tracking_cb(const ros::TimerEvent& e)
{
    if(!detected_by_myself || exec_state != EXEC_STATE::OBEJECT_TRACKING)
    {
        return;
    }

    // 静态目标 直接飞至上空即可
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::SwarmCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Move_mode           = prometheus_msgs::SwarmCommand::XYZ_POS;
    Command_Now.position_ref[0]     = object_pos_enu[0];
    Command_Now.position_ref[1]     = object_pos_enu[1];
    Command_Now.position_ref[2]     = fly_height_2D;
    Command_Now.yaw_ref             = 0.0;
    command_pub.publish(Command_Now);

}


// 主循环 
void Global_Planner::mainloop_cb(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;

    // 检查当前状态，不满足规划条件则直接退出主循环
    if(!odom_ready || !drone_ready || !sensor_ready || !station_ready)
    {
        // 此处改为根据循环时间计算的数值
        if(exec_num == 10)
        {
            if(!odom_ready)
            {
                message = "Need Odom.";
            }else if(!drone_ready)
            {
                message = "Drone is not ready.";
            }else if(!sensor_ready)
            {
                message = "Need sensor info.";
            }else if(!station_ready)
            {
                message = "Need station start cmd.";
            }

            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
            exec_num=0;
        }  

        return;
    }else
    {
        // 对检查的状态进行重置,此处要求无人机状态、传感器信息回调频率要高于本定时器
        odom_ready = false;
        drone_ready = false;
        sensor_ready = false;

        if(exec_num >= 5)
        {
            // 状态打印
            printf_exec_state();
            exec_num=0;
        }  

    }
    
    switch (exec_state)
    {
        case EXEC_STATE::INIT:            
            //　仿真模式
            if(sim_mode)
            {
                // 解锁
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Mode  = prometheus_msgs::SwarmCommand::Idle;
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.yaw_ref = 999;
                command_pub.publish(Command_Now);   
                ros::Duration(3.0).sleep();
                
                exec_state = EXEC_STATE::TAKEOFF;
            }else
            {
                //　真实飞行情况：等待飞机状态变为offboard模式，然后发送起飞指令
                // offboard指令由遥控器发出
                if(_DroneState.mode != "OFFBOARD")
                {
                    message = "Waiting for the offboard mode";
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
                    
                    ros::Duration(1.0).sleep();
                }else
                {
                    exec_state = EXEC_STATE::TAKEOFF;
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
                }

            }
            break;

        case EXEC_STATE::TAKEOFF:

            // 发布起飞指令
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode = prometheus_msgs::SwarmCommand::Takeoff;
            Command_Now.Command_ID = Command_Now.Command_ID + 1;
            Command_Now.yaw_ref = 0.0;
            Command_Now.source = NODE_NAME;
            command_pub.publish(Command_Now);
            
            // 起飞的位置设置为返航点
            return_pos = start_pos;
            char message_chars[256];
            sprintf(message_chars, "Takeoff and return point is set as [%f, %f, %f].", return_pos(0),return_pos(1),return_pos(2));
            message = message_chars;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);

            ros::Duration(5.0).sleep();

            exec_state = EXEC_STATE::WAIT_GOAL;

            break;

        case EXEC_STATE::WAIT_GOAL:
            // 等待目标点，不执行路径追踪逻辑
            path_ok = false;
            distance_walked = 0;     
            uav_pos_last = start_pos;       

            // 自动目标点模式：从目标点矩阵中读取目标点
            if(!manual_mode)
            {
                // goal_id初始设定为0，每执行一次+1，一共执行goal_num个目标点
                if(goal_id < goal_num)
                {
                    ros::Duration(3.0).sleep();
                    // 从目标点矩阵中提取目标点
                    goal_pos[0] = waypoints_[goal_id][0];
                    goal_pos[1] = waypoints_[goal_id][1];
                    goal_pos[2] = waypoints_[goal_id][2];
                    goal_id++;
                    // 目标点赋值成功，进入规划模式
                    exec_state = EXEC_STATE::PLANNING;
                }else
                {
                    // 执行完所有目标点，则进入返航模式
                    exec_state = EXEC_STATE::RETURN;
                }

            }
            // 手动目标点模式
            else
            {
                // 等待手动输入的目标值
                if(!get_goal)
                {
                    if(exec_num == 10)
                    {
                        message = "Waiting for a new goal.";
                        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,message);
                        exec_num=0;
                    }
                }else
                {
                    // 获取到目标点后，生成新轨迹
                    exec_state = EXEC_STATE::PLANNING;
                    get_goal = false;
                }
            }

            break;
        
        case EXEC_STATE::PLANNING:
            // 重置规划器
            Astar_ptr->reset();
            distance_walked = 0;     
            uav_pos_last = start_pos;  
            // 使用规划器执行搜索，返回搜索结果
            astar_state = Astar_ptr->search(start_pos, goal_pos);

            // 未寻找到路径
            if(astar_state==Astar::NO_PATH)
            {
                path_ok = false;
                // 找不到路径：返回等待目标点，若在自动目标点模式，则会前往下一个目标点
                exec_state = EXEC_STATE::WAIT_GOAL;
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Planner can't find path!");
            }
            else
            {
                path_ok = true;
                is_new_path = true;
                path_cmd = Astar_ptr->get_ros_path();
                // 路径中航点数目
                Num_total_wp = path_cmd.poses.size();
                // 计算从第几个点开始追踪
                start_point_index = get_start_point_id();
                cur_id = start_point_index;
                tra_start_time = ros::Time::now();
                // 路径规划成功，进入PATH_TRACKING
                exec_state = EXEC_STATE::PATH_TRACKING;
                // 发布路劲用于rviz显示
                path_cmd_pub.publish(path_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Get a new path!");       
            }

            break;
        
        case EXEC_STATE::PATH_TRACKING:
        
            // 执行时间达到阈值 或者 运动路径达到阈值，重新执行一次规划
            if(get_time_in_sec(tra_start_time) >= replan_time || distance_walked > 1.0)
            {
                exec_state = EXEC_STATE::PLANNING;
            }

            break;
        
        case EXEC_STATE::OBEJECT_TRACKING:
            
            path_ok = false;

            // 设定退出条件，return或降落
            // 也可以不设定，等待地面站消息

            break;

        case EXEC_STATE::RETURN_PLANNING:
            // 重置规划器
            Astar_ptr->reset();
            distance_walked = 0;     
            uav_pos_last = start_pos;  
            // 使用规划器执行搜索，返回搜索结果
            astar_state = Astar_ptr->search(start_pos, return_pos);
            // 未寻找到路径
            if(astar_state==Astar::NO_PATH)
            {
                path_ok = false;
                // 找不到路径：直接原地降落
                exec_state = EXEC_STATE::LAND;
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Planner can't find path, LAND!");
            }
            else
            {
                path_ok = true;
                is_new_path = true;
                path_cmd = Astar_ptr->get_ros_path();
                Num_total_wp = path_cmd.poses.size();
                start_point_index = get_start_point_id();
                cur_id = start_point_index;
                tra_start_time = ros::Time::now();
                path_cmd_pub.publish(path_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Return: Get a new path!");       
            }
            break;

        case EXEC_STATE::RETURN:
            // 执行时间达到阈值 或者 运动路径达到阈值，重新执行一次规划
            if(get_time_in_sec(tra_start_time) >= replan_time || distance_walked > 2.0)
            {
                exec_state = EXEC_STATE::RETURN_PLANNING;
            }
            
            // 抵达返航点附近，降落
            if( (start_pos - return_pos).norm() < 0.2)
            {
                exec_state = EXEC_STATE::LAND;
            }

            break;

        case EXEC_STATE::LAND:
            path_ok = false;
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode         = prometheus_msgs::SwarmCommand::Land;
            Command_Now.Command_ID   = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            command_pub.publish(Command_Now);
            break;
        
    }

}

// 【获取当前时间函数】 单位：秒
float Global_Planner::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void Global_Planner::printf_exec_state()
{
    char message_chars[256];
    switch (exec_state)
    {
        case EXEC_STATE::INIT: 
            sprintf(message_chars, "Exec_state [INIT].");
            break;
        case EXEC_STATE::TAKEOFF:
            sprintf(message_chars, "Exec_state [TAKEOFF].");
            break;
        case EXEC_STATE::WAIT_GOAL:
            sprintf(message_chars, "Exec_state [WAIT_GOAL].");
            break;
        case EXEC_STATE::PLANNING: 
            sprintf(message_chars, "Exec_state [PLANNING].");
            break;
        case EXEC_STATE::PATH_TRACKING:
            sprintf(message_chars, "Exec_state [PATH_TRACKING].");
            break;
        case EXEC_STATE::OBEJECT_TRACKING:
            sprintf(message_chars, "Exec_state [OBEJECT_TRACKING].");
            break;
        case EXEC_STATE::RETURN_PLANNING: 
            sprintf(message_chars, "Exec_state [RETURN_PLANNING].");
            break;
        case EXEC_STATE::RETURN:
            sprintf(message_chars, "Exec_state [RETURN].");
            break;
        case EXEC_STATE::LAND:
            sprintf(message_chars, "Exec_state [LAND].");
            break;  
    }    
    message = message_chars;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message); 


    if(detected_by_myself)
    {
        sprintf(message_chars, "detected_by_myself, the target pos is: [%f, %f, %f].",object_pos_enu[0],object_pos_enu[1],object_pos_enu[2]);
    }else if(detected_by_others)
    {
        sprintf(message_chars, "detected_by_others, return.");
    }else
    {
        sprintf(message_chars, "no one find the target, keep searching.");
    }
    message = message_chars;
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
}



int Global_Planner::get_start_point_id(void)
{
    // 选择与当前无人机所在位置最近的点,并从该点开始追踪
    int id = 0;
    float distance_to_wp_min = abs(path_cmd.poses[0].pose.position.x - _DroneState.position[0])
                                + abs(path_cmd.poses[0].pose.position.y - _DroneState.position[1])
                                + abs(path_cmd.poses[0].pose.position.z - _DroneState.position[2]);
    
    float distance_to_wp;

    for (int j=1; j<Num_total_wp;j++)
    {
        distance_to_wp = abs(path_cmd.poses[j].pose.position.x - _DroneState.position[0])
                                + abs(path_cmd.poses[j].pose.position.y - _DroneState.position[1])
                                + abs(path_cmd.poses[j].pose.position.z - _DroneState.position[2]);
        
        if(distance_to_wp < distance_to_wp_min)
        {
            distance_to_wp_min = distance_to_wp;
            id = j;
        }
    }

    //　为防止出现回头的情况，此处对航点进行前馈处理
    if(id + 1 < Num_total_wp)
    {
        id = id + 1;
    }

    return id;
}






}