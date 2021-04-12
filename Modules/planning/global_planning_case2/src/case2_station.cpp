//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include "prometheus_msgs/Case2Result.h"
#include "prometheus_msgs/StationCommandCase2.h"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "case2_station");
    ros::NodeHandle nh("~");
    
    // 订阅case2发布的结果

    // 发布指令至无人机
}