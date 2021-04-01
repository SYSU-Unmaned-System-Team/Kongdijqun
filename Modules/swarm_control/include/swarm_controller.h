#ifndef SWARM_CONTROLLER_H
#define SWARM_CONTROLLER_H
#include <ros/ros.h>
#include <bitset>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>

#include "swarm_control_utils.h"
#include "message_utils.h"
#include <math_utils.h>

// 
#define NODE_NAME "swarm_controller"
using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int swarm_num;
string uav_name;
int num_neighbour = 2;
int uav_id,neighbour_id1,neighbour_id2;
string neighbour_name1,neighbour_name2;

float Takeoff_height;                                       //默认起飞高度
float Disarm_height;                                        //自动上锁高度
float Land_speed;                                           //降落速度
Eigen::MatrixXf formation_separation;

// 速度控制参数
float k_p;
float k_aij;
float k_gamma;

bool flag_printf;

//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;
Eigen::Vector3f gazebo_offset;

Eigen::Vector3d Takeoff_position;                              // 起飞位置
prometheus_msgs::DroneState _DroneState;                         //无人机状态量

Eigen::Vector3d pos_drone;
Eigen::Vector3d vel_drone;

Eigen::Vector3d pos_nei[2];
Eigen::Vector3d vel_nei[2];

prometheus_msgs::SwarmCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::SwarmCommand Command_Last;                     //无人机上一条执行命令

Eigen::Vector3d state_sp(0,0,0);
Eigen::Vector3d state_sp_extra(0,0,0);
float yaw_sp;

float yita;
Eigen::Vector3d accel_sp;
Eigen::Vector3d throttle_sp;

prometheus_msgs::Message message;

float dt = 0;


// 订阅
ros::Subscriber command_sub;
ros::Subscriber drone_state_sub;
ros::Subscriber position_target_sub;
ros::Subscriber nei1_state_sub;
ros::Subscriber nei2_state_sub;

// 发布
ros::Publisher setpoint_raw_local_pub;
ros::Publisher message_pub;

// 服务
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;
namespace swarm_controller 
{
void init()
{


}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> swarm controller Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "uav_name   : "<< uav_name <<endl;
    cout << "neighbour_name1   : "<< neighbour_name1 <<endl;
    cout << "neighbour_name2   : "<< neighbour_name2 <<endl;
    cout << "k_p    : "<< k_p <<"  "<<endl;
    cout << "k_aij       : "<< k_aij <<"  "<<endl;
    cout << "k_gamma       : "<< k_gamma <<"  "<<endl;
    

    cout << "Takeoff_height   : "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height    : "<< Disarm_height <<" [m] "<<endl;
    cout << "Land_speed       : "<< Land_speed <<" [m/s] "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;
}

void printf_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Swarm Controller  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << "UAV_id : " <<  uav_id << "   UAV_name : " <<  uav_name << endl;
    cout << "neighbour_id1 : " <<  neighbour_id1 << "   neighbour_name1 : " <<  neighbour_name1 << endl;
    cout << "neighbour_id2 : " <<  neighbour_id2 << "   neighbour_name2 : " <<  neighbour_name2 << endl;
    cout << "UAV_pos [X Y Z] : " << pos_drone[0] << " [ m ] "<< pos_drone[1]<<" [ m ] "<<pos_drone[2]<<" [ m ] "<<endl;
    cout << "UAV_vel [X Y Z] : " << vel_drone[0] << " [ m/s ] "<< vel_drone[1]<<" [ m/s ] "<<vel_drone[2]<<" [ m/s ] "<<endl;
    cout << "neighbour_pos [X Y Z] : " << pos_nei[0][0] << " [ m ] "<< pos_nei[0][1]<<" [ m ] "<<pos_nei[0][2]<<" [ m ] "<<endl;
    cout << "neighbour_vel [X Y Z] : " << vel_nei[0][0] << " [ m/s ] "<< vel_nei[0][1]<<" [ m/s ] "<<vel_nei[0][2]<<" [ m/s ] "<<endl;
    cout << "neighbour_pos [X Y Z] : " << pos_nei[1][0] << " [ m ] "<< pos_nei[1][1]<<" [ m ] "<<pos_nei[1][2]<<" [ m ] "<<endl;
    cout << "neighbour_vel [X Y Z] : " << vel_nei[1][0] << " [ m/s ] "<< vel_nei[1][1]<<" [ m/s ] "<<vel_nei[1][2]<<" [ m/s ] "<<endl;
}


int check_failsafe()
{
    if (_DroneState.position[0] < geo_fence_x[0] || _DroneState.position[0] > geo_fence_x[1] ||
        _DroneState.position[1] < geo_fence_y[0] || _DroneState.position[1] > geo_fence_y[1] ||
        _DroneState.position[2] < geo_fence_z[0] || _DroneState.position[2] > geo_fence_z[1])
    {
        pub_message(message_pub, prometheus_msgs::Message::ERROR, NODE_NAME, "Out of the geo fence, the drone is landing...");
        return 1;
    }
    else{
        return 0;
    }
}


void timerCallback(const ros::TimerEvent& e)
{
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "[" + uav_name + "] : Program is running.");
}

void idle()
{
    mavros_msgs::PositionTarget pos_setpoint;

    //飞控如何接收该信号请见mavlink_receiver.cpp
    //飞控如何执行该指令请见FlightTaskOffboard.cpp
    pos_setpoint.type_mask = 0x4000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送位置期望值至飞控（输入：期望xyz,期望yaw）
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
void send_vel_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}


void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d& state_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b100111000011;   // 100 111 000 011  vx vy vz z + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = state_sp[0];
    pos_setpoint.velocity.y = state_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = state_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
    
    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

void send_pos_vel_xyz_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 速度作为前馈项， 参见FlightTaskOffboard.cpp
    // 2. position setpoint + velocity setpoint (velocity used as feedforward)
    // 控制方法请见 PositionControl.cpp
    pos_setpoint.type_mask = 0b100111000000;   // 100 111 000 000  vx vy　vz x y z+ yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送加速度期望值至飞控（输入：期望axayaz,期望yaw）
void send_acc_xyz_setpoint(const Eigen::Vector3d& accel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100000111111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.acceleration_or_force.x = accel_sp[0];
    pos_setpoint.acceleration_or_force.y = accel_sp[1];
    pos_setpoint.acceleration_or_force.z = accel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Acc_target [X Y Z] : " << accel_drone_fcu_target[0] << " [m/s^2] "<< accel_drone_fcu_target[1]<<" [m/s^2] "<<accel_drone_fcu_target[2]<<" [m/s^2] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;

}
}

#endif


