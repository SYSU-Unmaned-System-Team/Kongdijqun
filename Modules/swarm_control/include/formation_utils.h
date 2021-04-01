#ifndef FORMATION_UTILS_H
#define FORMATION_UTILS_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <prometheus_msgs/Message.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>
#include <prometheus_msgs/LogMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

namespace formation_utils 
{
// 输入参数：　阵型，阵型基本尺寸，集群数量
// 所有的阵型和数量必须提前预设!!
Eigen::MatrixXf get_formation_separation(int swarm_shape, float swarm_size, int swarm_num)
{
    //矩阵大小为　swarm_num＊4 , 对应　x,y,z,yaw四个自由度的分离量
    Eigen::MatrixXf seperation(swarm_num,4); 


    if(swarm_num == 1)
    {
        seperation(0,0) = 0.0;
        seperation(0,1) = 0.0;  
        seperation(0,2) = 0.0;
        seperation(0,3) = 0.0;
    }
 
    // cxy 默认swarm_size为１米

    // one_column shape
    // 横向一字型，虚拟领机位置为中心位置，其余飞机根据数量向左右增加
    if(swarm_shape == prometheus_msgs::SwarmCommand::One_column)
    {
        if(swarm_num == 8)
        {
            seperation(0,0) = 0.5 * swarm_size;
            seperation(0,1) = 0.0 * swarm_size;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = -0.5 * swarm_size;
            seperation(1,1) = 0.0 * swarm_size;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 1.5 * swarm_size;
            seperation(2,1) = 0.0 * swarm_size;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;

            seperation(3,0) = -1.5 * swarm_size;
            seperation(3,1) = 0.0 * swarm_size;  
            seperation(3,2) = 0.0;
            seperation(3,3) = 0.0;

            seperation(4,0) = 2.5 * swarm_size;
            seperation(4,1) = 0.0 * swarm_size;  
            seperation(4,2) = 0.0;
            seperation(4,3) = 0.0;

            seperation(5,0) = -2.5 * swarm_size;
            seperation(5,1) = 0.0 * swarm_size;  
            seperation(5,2) = 0.0;
            seperation(5,3) = 0.0;

            seperation(6,0) = 3.5 * swarm_size;
            seperation(6,1) = 0.0 * swarm_size;  
            seperation(6,2) = 0.0;
            seperation(6,3) = 0.0;

            seperation(7,0) = -3.5 * swarm_size;
            seperation(7,1) = 0.0 * swarm_size;  
            seperation(7,2) = 0.0;
            seperation(7,3) = 0.0;
        }
    }

    // triangle shape
    // 三角型，虚拟领机位置为中心位置
    if(swarm_shape == prometheus_msgs::SwarmCommand::Triangle)
    {
        if(swarm_num == 8)
        {
            seperation(0,0) = 0.5 * swarm_size;
            seperation(0,1) = 2.0 * swarm_size;  
            seperation(0,2) = 00;
            seperation(0,3) = 0.0;

            seperation(1,0) = -0.5 * swarm_size;
            seperation(1,1) = 2.0 * swarm_size;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 1.5 * swarm_size;
            seperation(2,1) = 1.0 * swarm_size;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;

            seperation(3,0) = -1.5 * swarm_size;
            seperation(3,1) = 1.0 * swarm_size;  
            seperation(3,2) = 0.0;
            seperation(3,3) = 0.0;

            seperation(4,0) = 2.5 * swarm_size;
            seperation(4,1) = -0.0 * swarm_size;  
            seperation(4,2) = 0.0;
            seperation(4,3) = 0.0;

            seperation(5,0) = -2.5 * swarm_size;
            seperation(5,1) = 0.0 * swarm_size;  
            seperation(5,2) = 0.0;
            seperation(5,3) = 0.0;

            seperation(6,0) = 3.5 * swarm_size;
            seperation(6,1) = -1.0 * swarm_size;  
            seperation(6,2) = 0.0;
            seperation(6,3) = 0.0;

            seperation(7,0) = -3.5 * swarm_size;
            seperation(7,1) = -1.0 * swarm_size;  
            seperation(7,2) = 0.0;
            seperation(7,3) = 0.0;
        }
    }

    // Square shape
    // 方型，虚拟领机位置为中心位置
    if(swarm_shape == prometheus_msgs::SwarmCommand::Square)
    {
        if(swarm_num == 8)
        {
            seperation(0,0) = 0.5 * swarm_size;
            seperation(0,1) = -2.0 * swarm_size;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = -0.5 * swarm_size;
            seperation(1,1) = 2.0 * swarm_size;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 2.5 * swarm_size;
            seperation(2,1) = 2.0 * swarm_size;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;

            seperation(3,0) = -2.5 * swarm_size;
            seperation(3,1) = 2.0 * swarm_size;  
            seperation(3,2) = 0.0;
            seperation(3,3) = 0.0;

            seperation(4,0) = 2.5 * swarm_size;
            seperation(4,1) = 0.0 * swarm_size;  
            seperation(4,2) = 0.0;
            seperation(4,3) = 0.0;

            seperation(5,0) = -2.5 * swarm_size;
            seperation(5,1) = 0.0 * swarm_size;  
            seperation(5,2) = 0.0;
            seperation(5,3) = 0.0;

            seperation(6,0) = 2.5 * swarm_size;
            seperation(6,1) = -2.0 * swarm_size;  
            seperation(6,2) = 0.0;
            seperation(6,3) = 0.0;

            seperation(7,0) = -2.5 * swarm_size;
            seperation(7,1) = -2.0 * swarm_size;  
            seperation(7,2) = 0.0;
            seperation(7,3) = 0.0;
        }
    }

    // Circular shape
    // 圆形，虚拟领机位置为中心位置
    if(swarm_shape == prometheus_msgs::SwarmCommand::Circular)
    {
        if(swarm_num == 8)
        {
            seperation(0,0) = 0.5 * swarm_size;
            seperation(0,1) = -1.0 * swarm_size;  
            seperation(0,2) = 0.0;
            seperation(0,3) = 0.0;

            seperation(1,0) = -0.5 * swarm_size;
            seperation(1,1) = 1.0 * swarm_size;  
            seperation(1,2) = 0.0;
            seperation(1,3) = 0.0;

            seperation(2,0) = 2.0 * swarm_size;
            seperation(2,1) = 1.0 * swarm_size;  
            seperation(2,2) = 0.0;
            seperation(2,3) = 0.0;

            seperation(3,0) = -2.0 * swarm_size;
            seperation(3,1) = 1.0 * swarm_size;  
            seperation(3,2) = 0.0;
            seperation(3,3) = 0.0;

            seperation(4,0) = 2.0 * swarm_size;
            seperation(4,1) = -1.0 * swarm_size;  
            seperation(4,2) = 0.0;
            seperation(4,3) = 0.0;

            seperation(5,0) = -2.0 * swarm_size;
            seperation(5,1) = -1.0 * swarm_size;  
            seperation(5,2) = 0.0;
            seperation(5,3) = 0.0;

            seperation(6,0) = 3.5 * swarm_size;
            seperation(6,1) = 0.0 * swarm_size;  
            seperation(6,2) = 0.0;
            seperation(6,3) = 0.0;

            seperation(7,0) = -3.5 * swarm_size;
            seperation(7,1) = 0.0 * swarm_size;  
            seperation(7,2) = 0.0;
            seperation(7,3) = 0.0;
        }
    }
    return seperation;
}

}
#endif
