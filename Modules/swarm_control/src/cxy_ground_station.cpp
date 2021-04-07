//头文件
#include <cxy_ground_station_utils.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cxy_ground_station");
    ros::NodeHandle nh("~");

    // 参数读取
    nh.param<int>("swarm_num", swarm_num, 1);
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<string>("uav_name", uav_name, "/uav1");
    nh.param<float>("refresh_time", refresh_time, 1.0);

    // 根据任务类型 订阅不同话题,打印不同话题
    nh.param<int>("mission_type", mission_type, MISSION_TYPE::TEST);

    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 1, drone_state_cb);

    command_sub = nh.subscribe<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 1, swarm_command_cb);

    //message_sub = nh.subscribe<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10, msg_cb);

    // 频率
    float hz = 1.0 / refresh_time;
    ros::Rate rate(hz);


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        cout <<">>>>>>>>>>>>>>>>>>>>>>>> CXY Ground Station  <<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;


        //打印无人机状态
        prinft_drone_state(uav_id, _DroneState);

        printf_swarm_command(Command_Now);

        rate.sleep();
    }

    return 0;

}

