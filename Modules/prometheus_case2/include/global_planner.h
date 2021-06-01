#ifndef GLOBAL_PLANNER
#define GLOBAL_PLANNER

#include <ros/ros.h>
#include <boost/format.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/Message.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/SwarmCommand.h"
#include "prometheus_msgs/ArucoInfo.h"
#include "prometheus_msgs/Case2Result.h"
#include "prometheus_msgs/StationCommandCase2.h"

#include "A_star.h"
#include "occupy_map.h"

using namespace std;

#define MIN_DIS 0.1

namespace Global_Planning
{

class Global_Planner
{

private:
    // ros nh
    ros::NodeHandle global_planner_nh;
    // 订阅地面站指令
    ros::Subscriber station_cmd_sub;
    // 订阅目标点
    ros::Subscriber goal_sub;
    // 订阅无人机状态
    ros::Subscriber drone_state_sub;
    // 订阅传感器数据
    ros::Subscriber Gpointcloud_sub;
    ros::Subscriber Lpointcloud_sub;
    ros::Subscriber laserscan_sub;
    // 订阅识别结果
    ros::Subscriber detection_sub;
    // 发布控制指令
    ros::Publisher command_pub,path_cmd_pub;
    // 发布检测结果至其他无人机、无人车、地面站
    ros::Publisher case2_result_pub;
    // 主循环定时器、路径追踪定时器、目标追踪定时器
    ros::Timer mainloop_timer, track_path_timer, object_tracking_timer;
    // 五种状态机
    enum EXEC_STATE
    {
        INIT,
        TAKEOFF,
        WAIT_GOAL,
        PLANNING,
        PATH_TRACKING,
        OBEJECT_TRACKING,
        RETURN_PLANNING,
        RETURN,
        LAND,
    };
    EXEC_STATE exec_state;
    // 集群数量
    int swarm_num;     
    // 无人机名字                             
    string uav_name;     
    // 节点名字                           
    string node_name;  
    // 是否仿真模式
    bool sim_mode;    
    // 无人机编号                         
    int uav_id;                                     
    // 飞行高度
    double fly_height_2D;
    // 手动给定目标点模式 或 自动目标点模式
    bool manual_mode;
    // 自动目标点数量
    int goal_num;
    // 自动目标点矩阵
    double waypoints_[50][3];
    // 自动目标点id
    int goal_id;
    // 传感器输入flag
    int map_input_source;
    // 路径重规划时间
    double replan_time;
    // 控制器flag
    int control_flag;
    // 每段路径的预设时间（速度追踪模式时）
    double time_per_path;
    // 最大追踪速度
    double vel_max;

    // A星规划器
    Astar::Ptr Astar_ptr;
    // A星规划器状态
    int astar_state;
    // 无人机状态
    prometheus_msgs::DroneState _DroneState;
    // 地面站指令
    prometheus_msgs::StationCommandCase2 station_cmd;
    // 发布的控制指令
    prometheus_msgs::SwarmCommand Command_Now;
    // 无人机里程计信息
    nav_msgs::Odometry Drone_odom;
    // 规划得到的路径
    nav_msgs::Path path_cmd;
    // 路经点开始id
    int start_point_index;
    // 路经点总数
    int Num_total_wp;
    // 当前执行ID
    int cur_id;
    // 距离上一次重置，移动的距离（重规划路径时重置）
    float distance_walked;
    // 上一次重置时，无人机的位置
    Eigen::Vector3d uav_pos_last;
    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, start_vel, start_acc, goal_pos, goal_vel;
    // 返航位置
    Eigen::Vector3d return_pos;
    // 规划器状态
    bool odom_ready;
    bool drone_ready;
    bool sensor_ready;
    bool station_ready;
    bool get_goal; 
    bool is_new_path;
    bool path_ok;
    // 期望偏航角，一般锁定为0
    float desired_yaw;
    // 上一条轨迹开始时间
    ros::Time tra_start_time;    

    // 检测相关状态量
    bool detected_by_myself;
    bool detected_by_others;
    // 是否丢失目标
    bool lost_object;
    int num_count_vision_lost;
    int num_count_vision_get;
    // 目标位置
    Eigen::Vector3d object_pos_body,object_pos_enu;
    // 检测范围
    Eigen::Vector2d detection_range_x,detection_range_y;
    //打印颜色设置
    string red, green, yellow, tail;

    // 回调函数
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void drone_state_cb(const prometheus_msgs::DroneStateConstPtr &msg);
    void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);
    void detection_cb(const prometheus_msgs::ArucoInfoConstPtr &msg);
    void cmd_cb(const prometheus_msgs::StationCommandCase2ConstPtr& msg);
    
    void mainloop_cb(const ros::TimerEvent& e);
    void track_path_cb(const ros::TimerEvent& e);
    void object_tracking_cb(const ros::TimerEvent& e);
   
    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const ros::Time& begin_time);
    int get_start_point_id(void);
    void printf_exec_state();
    
public:
    Global_Planner(void):
        global_planner_nh("~")
    {}~Global_Planner(){}

    void init(ros::NodeHandle& nh);
};

}

#endif