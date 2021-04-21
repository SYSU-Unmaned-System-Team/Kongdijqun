#include <ros/ros.h>
#include <iostream>
#include "prometheus_msgs/StationCommandCase2.h"

using namespace std;

bool sim_mode;
bool flag_ros2groundstation;
prometheus_msgs::StationCommandCase2 case2_cmd;
ros::Publisher case2_cmd_pub;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "case2_from_groundstation");
    ros::NodeHandle nh("~");

    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_ros2groundstation", flag_ros2groundstation, false);

    case2_cmd_pub = nh.advertise<prometheus_msgs::StationCommandCase2>("/case2/command_uav", 1); 

    // 初始化
    case2_cmd.Command_uav = prometheus_msgs::StationCommandCase2::Start;
    case2_cmd.detection_flag = prometheus_msgs::StationCommandCase2::no_one_detected;
    case2_cmd.enu_position[0] = 0.0;
    case2_cmd.enu_position[1] = 0.0;
    case2_cmd.enu_position[2] = 0.0;

    // 仿真模式下：所有指令通过终端输入（取代实际地面站发来的消息）
    if(sim_mode)
    {
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>CASE 2 Station<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please enter 1 to start case2 mission."<<endl;
            cin >> start_flag;

            // case2 接收到该消息后才会启动
            case2_cmd.Command_uav = prometheus_msgs::StationCommandCase2::Start;
            case2_cmd_pub.publish(case2_cmd);
        }

        while(ros::ok())
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>CASE 2 Station<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please enter 1 for Return, 2 for Land."<<endl;
            cin >> start_flag;

            switch(start_flag)
            {
                case 1:
                    case2_cmd.Command_uav = prometheus_msgs::StationCommandCase2::Return;
                    case2_cmd_pub.publish(case2_cmd);
                    break;
                case 2:
                    case2_cmd.Command_uav = prometheus_msgs::StationCommandCase2::Land;
                    case2_cmd_pub.publish(case2_cmd);
                    break;
            }
        }
    }else
    {
        // 非sim_mode， 即所有指令来自真实地面站，根据不同指令发送不同消息
        /* code */

    }
    
  

    return 0;
}