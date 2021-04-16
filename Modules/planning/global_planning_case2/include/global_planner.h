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
#include "tools.h"
#include "message_utils.h"

using namespace std;

#define NODE_NAME "Global_Planner [main]"

#define MIN_DIS 0.1

namespace Global_Planning
{

extern ros::Publisher message_pub;

class Global_Planner
{
private:

    ros::NodeHandle global_planner_nh;
    int swarm_num;                                  // 集群数量
    string uav_name;                                // 无人机名字
    int uav_id;                                     // 无人机编号
    // 参数
    int algorithm_mode;
    double fly_height_2D;
    double time_per_path;
    int map_input;
    double replan_time;
    bool consider_neighbour;
    bool sim_mode;
    

    // 本机位置
    // 邻机位置
    // 根据不同的输入（激光雷达输入、相机输入等）生成occupymap
    // 调用路径规划算法 生成路径
    // 调用轨迹优化算法 规划轨迹

     
    ros::Subscriber station_cmd_sub;
    // 订阅无人机状态、目标点、传感器数据（生成地图）
    ros::Subscriber goal_sub;
    ros::Subscriber drone_state_sub;
    // 支持2维激光雷达、3维激光雷达、D435i等实体传感器
    // 支持直接输入全局已知点云
    ros::Subscriber Gpointcloud_sub;
    ros::Subscriber Lpointcloud_sub;
    ros::Subscriber laserscan_sub;
    // 订阅识别结果
    ros::Subscriber detection_sub;

    // 发布控制指令
    ros::Publisher command_pub,path_cmd_pub;
    // 发布检测结果至其他无人机 无人车 地面站
    ros::Publisher case2_result_pub;
    ros::Timer mainloop_timer, track_path_timer, object_tracking_timer;

    // A星规划器
    Astar::Ptr Astar_ptr;
    int astar_state;

    prometheus_msgs::DroneState _DroneState;
    prometheus_msgs::StationCommandCase2 station_cmd;
    
    nav_msgs::Odometry Drone_odom;

    nav_msgs::Path path_cmd;


    float distance_walked;
    prometheus_msgs::SwarmCommand Command_Now;   

    Eigen::Vector3d uav_pos_last;
    double distance_to_goal;

    // 规划器状态
    bool odom_ready;
    bool drone_ready;
    bool sensor_ready;
    bool get_goal; 
    bool is_new_path;
    bool path_ok;
    int start_point_index;
    int Num_total_wp;
    int cur_id;

    // 自动目标点相关
    bool manual_mode;
    int goal_num;
    Eigen::MatrixXf goal_matrix;
    int goal_id;

    // 返航相关
    Eigen::Vector3d return_pos;


    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, start_vel, start_acc, goal_pos, goal_vel;

    float desired_yaw;

    ros::Time tra_start_time;
    float tra_running_time;
    
    // 打印的提示消息
    string message;

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

    // 检测相关
    bool detected_by_myself;
    bool detected_by_others;
    bool lost_object;
    int num_count_vision_lost;
    int num_count_vision_get;
    Eigen::Vector3d object_pos_body,object_pos_enu;
    Eigen::Vector2d detection_range_x,detection_range_y;


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
    
public:
    Global_Planner(void):
        global_planner_nh("~")
    {}~Global_Planner(){}

    void init(ros::NodeHandle& nh);
};

}

#endif