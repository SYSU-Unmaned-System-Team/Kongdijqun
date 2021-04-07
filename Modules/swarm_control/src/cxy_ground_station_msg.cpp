//头文件
#include <ros/ros.h>
#include <prometheus_msgs/Message.h>

#include "message_utils.h"

void msg_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;
    printf_message(message);
    sleep(0.2);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cxy_ground_station_msg");
    ros::NodeHandle nh("~");

    string uav_name;

    nh.param<string>("uav_name", uav_name, "/uav1");


    ros::Subscriber message_sub = nh.subscribe<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10, msg_cb);

    // 频率
    float hz = 1.0;
    ros::Rate rate(hz);


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}

