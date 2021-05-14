#include <ros/ros.h>
#include <iostream>
#include "prometheus_msgs/StationCommandCase3.h"

using namespace std;

bool sim_mode;
bool flag_ros2groundstation;
prometheus_msgs::StationCommandCase3 case3_cmd;
ros::Publisher case3_cmd_pub;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "case3_from_groundstation");
    ros::NodeHandle nh("~");

    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_ros2groundstation", flag_ros2groundstation, false);

    case3_cmd_pub = nh.advertise<prometheus_msgs::StationCommandCase3>("/case3/command_uav", 1); 

    // 初始化
    case3_cmd.Command_uav = prometheus_msgs::StationCommandCase3::Start;
    case3_cmd.detection_flag = prometheus_msgs::StationCommandCase3::no_one_detected;
    case3_cmd.enu_position[0] = 0.0;
    case3_cmd.enu_position[1] = 0.0;
    case3_cmd.enu_position[2] = 0.0;

    // 仿真模式下：所有指令通过终端输入（取代实际地面站发来的消息）
    if(sim_mode)
    {
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>CASE 3 Station<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please enter 1 to start case3 mission."<<endl;
            cin >> start_flag;

            case3_cmd.Flag_id = 0b1111;
            case3_cmd.goal_uav1[0] = -2.0;
            case3_cmd.goal_uav1[1] = 2.0;
            case3_cmd.goal_uav1[2] = 1.0;

            case3_cmd.goal_uav2[0] = -1.0;
            case3_cmd.goal_uav2[1] = 2.0;
            case3_cmd.goal_uav2[2] = 1.0;

            case3_cmd.goal_uav3[0] = 1.0;
            case3_cmd.goal_uav3[1] = 2.0;
            case3_cmd.goal_uav3[2] = 1.0;

            case3_cmd.goal_uav4[0] = 2.0;
            case3_cmd.goal_uav4[1] = 2.0;
            case3_cmd.goal_uav4[2] = 1.0;

            case3_cmd.Command_uav = prometheus_msgs::StationCommandCase3::Start;
            case3_cmd_pub.publish(case3_cmd);
        }

        while(ros::ok())
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>CASE3 Station<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please enter 1 for Return, 2 for Land."<<endl;
            cin >> start_flag;

            switch(start_flag)
            {
                case 1:
                    case3_cmd.Command_uav = prometheus_msgs::StationCommandCase3::Return;
                    case3_cmd_pub.publish(case3_cmd);
                    break;
                case 2:
                    case3_cmd.Command_uav = prometheus_msgs::StationCommandCase3::Land;
                    case3_cmd_pub.publish(case3_cmd);
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