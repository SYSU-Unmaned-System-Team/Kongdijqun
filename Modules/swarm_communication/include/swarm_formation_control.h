#ifndef SWARM_ESTIMATOR_H
#define SWARM_ESTIMATOR_H

// 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <geometry_msgs/PoseStamped.h>
//socket头文件
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>

using namespace std;

// 宏定义
#define NODE_NAME "swarm_formation_control"
#define MAX_NUM 40
#define MAXLINE 128 
// 变量
int swarm_num;
string uav_name[MAX_NUM+1];
int uav_id[MAX_NUM+1];
prometheus_msgs::SwarmCommand swarm_command[MAX_NUM+1];
ros::Publisher command_pub[MAX_NUM+1];
ros::Subscriber drone_state_sub[MAX_NUM+1];
int enum_swarm_shape[5] = { prometheus_msgs::SwarmCommand::One_column,prometheus_msgs::SwarmCommand::Triangle,
                            prometheus_msgs::SwarmCommand::Square,prometheus_msgs::SwarmCommand::Circular};
int enum_controller_num[4] = {  prometheus_msgs::SwarmCommand::Position_Control,prometheus_msgs::SwarmCommand::Velocity_Control,
                                prometheus_msgs::SwarmCommand::Accel_Control};
string enum_swarm_shape_name[5] = {"One_column","Triangle","Square","Circular"};

int controller_num;
float formation_size;
bool sim_mode;
int start_flag;

Eigen::Vector3f virtual_leader_pos;
Eigen::Vector3f virtual_leader_vel;
float virtual_leader_yaw;

int formation_num = 1;

void pub_formation_command_for_uavx(int uav_id)
{
    if(formation_num>0 && formation_num<5)
    {
        swarm_command[uav_id].swarm_shape = enum_swarm_shape[formation_num-1];
    }
    else
    {
        printf("Wrong formation shape!\n");
    }
    if(controller_num>-1 && controller_num<3)
    {
        swarm_command[uav_id].Mode = enum_controller_num[controller_num];
    }
    else
    {
        printf("Wrong formation shape!\n");
    }
    swarm_command[uav_id].swarm_size = formation_size;
    swarm_command[uav_id].position_ref[0] = virtual_leader_pos[0] ; 
    swarm_command[uav_id].position_ref[1] = virtual_leader_pos[1] ;
    swarm_command[uav_id].position_ref[2] = virtual_leader_pos[2] ;  
    swarm_command[uav_id].velocity_ref[0] = virtual_leader_vel[0] ; 
    swarm_command[uav_id].velocity_ref[1] = virtual_leader_vel[1] ; 
    swarm_command[uav_id].velocity_ref[2] = virtual_leader_vel[2] ; 
    swarm_command[uav_id].yaw_ref = virtual_leader_yaw;
    command_pub[uav_id].publish(swarm_command[uav_id]);
}

void pub_formation_command()
{
    printf("Formation shape: [ %s ]",enum_swarm_shape_name[formation_num-1]);
    for(int i = 1; i <= swarm_num; i++) 
    {
        pub_formation_command_for_uavx(i);
    }
    printf("virtual_leader_pos: %fm %fm %fm\n",virtual_leader_pos[0],virtual_leader_pos[1],virtual_leader_pos[2]);
}

void printf_param()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Formation Flight Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "swarm_num   : "<< swarm_num <<endl;
    cout << "controller_num   : "<< controller_num <<endl;
    cout << "formation_size : "<< formation_size << " [m] "<< endl;

    for(int i = 1; i <= swarm_num; i++) 
    {
        cout << "uav_name["<< i << "]" << uav_name[i] <<endl;
        cout << "uav_id["<< i << "]" << uav_id[i] <<endl;
    }  
}

void disarm_swarm_for_uavx(int uav_id)
{
    swarm_command[uav_id].Mode = prometheus_msgs::SwarmCommand::Idle;
    swarm_command[uav_id].yaw_ref = 999;
    command_pub[uav_id].publish(swarm_command[uav_id]); //【发布】阵型
}

void disarm_swarm()
{
    for(int i = 1; i <= swarm_num; i++) 
    {
        disarm_swarm_for_uavx(i);
    }
}

void takeoff_swarm_for_uavx(int uav_id)
{
    swarm_command[uav_id].Mode = prometheus_msgs::SwarmCommand::Takeoff;
    swarm_command[uav_id].yaw_ref = 0.0;
    command_pub[uav_id].publish(swarm_command[uav_id]); //【发布】阵型  
}

void takeoff_swarm()
{
    for(int i = 1; i <= swarm_num; i++) 
    {
        takeoff_swarm_for_uavx(i);
    }
}

void land_swarm_for_uavx(int uav_id)
{
    swarm_command[uav_id].Mode = prometheus_msgs::SwarmCommand::Land;
    command_pub[uav_id].publish(swarm_command[uav_id]); //【发布】阵型    
}

void land_swarm()
{
    for(int i = 1; i <= swarm_num; i++) 
    {
        land_swarm_for_uavx(i);
    }
}

void arm_swarm_for_uavx(int uav_id)
{
    swarm_command[uav_id].Mode = prometheus_msgs::SwarmCommand::Disarm;
    command_pub[uav_id].publish(swarm_command[uav_id]); //【发布】阵型   
}

void arm_swarm()
{
    for(int i = 1; i <= swarm_num; i++) 
    {
        arm_swarm_for_uavx(i);
    }
}

#endif