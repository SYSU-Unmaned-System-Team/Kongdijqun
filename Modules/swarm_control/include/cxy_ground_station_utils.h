#ifndef CXY_GROUND_STATION_UTILS_H
#define CXY_GROUND_STATION_UTILS_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <math.h>
#include <prometheus_msgs/Message.h>
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/PositionReference.h>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/ControlOutput.h>
#include <prometheus_msgs/LogMessageControl.h>
#include <prometheus_msgs/LogMessageDetection.h>
#include <prometheus_msgs/LogMessagePlanning.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "message_utils.h"

using namespace std;

#define NUM_POINT 2

float refresh_time;
int mission_type;
int swarm_num;
int uav_id;
string uav_name;
ros::Subscriber command_sub;
ros::Subscriber drone_state_sub;
ros::Subscriber message_sub;


prometheus_msgs::DroneState _DroneState;
prometheus_msgs::SwarmCommand Command_Now;

// 五种状态机
enum MISSION_TYPE
{
    TEST,
    CASE1,
    CASE2,
    CASE3,
};


void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;
}

void swarm_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    Command_Now = *msg;
}

void msg_cb(const prometheus_msgs::Message::ConstPtr& msg)
{
    prometheus_msgs::Message message = *msg;
    printf_message(message);
    sleep(0.2);
}

// 打印无人机状态
void prinft_drone_state(int uav_id, const prometheus_msgs::DroneState& _Drone_state)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>> No.["<< uav_id <<"] "<<" State   <<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

    cout << "Time: " << _Drone_state.time_from_start <<" [s] ";

    //是否和飞控建立起连接
    if (_Drone_state.connected == true)
    {
        cout << " [ Connected ]";
    }
    else
    {
        cout << " [ Unconnected ]";
    }

    //是否上锁
    if (_Drone_state.armed == true)
    {
        cout << " [ Armed ]";
    }
    else
    {
        cout << " [ DisArmed ]";
    }

    //是否在地面
    if (_Drone_state.landed == true)
    {
        cout << " [ Ground ] ";
    }
    else
    {
        cout << " [ Air ] ";
    }

    cout << "[ " << _Drone_state.mode<<" ] " <<endl;

    cout << "Position [X Y Z] : " << _Drone_state.position[0] << " [ m ] "<< _Drone_state.position[1]<<" [ m ] "<<_Drone_state.position[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << _Drone_state.velocity[0] << " [m/s] "<< _Drone_state.velocity[1]<<" [m/s] "<<_Drone_state.velocity[2]<<" [m/s] "<<endl;
    cout << "Attitude [R P Y] : " << _Drone_state.attitude[0] * 180/M_PI <<" [deg] "<<_Drone_state.attitude[1] * 180/M_PI << " [deg] "<< _Drone_state.attitude[2] * 180/M_PI<<" [deg] "<<endl;
    //cout << "Att_rate [R P Y] : " << _Drone_state.attitude_rate[0] * 180/M_PI <<" [deg/s] "<<_Drone_state.attitude_rate[1] * 180/M_PI << " [deg/s] "<< _Drone_state.attitude_rate[2] * 180/M_PI<<" [deg/s] "<<endl;
}


void printf_swarm_command(const prometheus_msgs::SwarmCommand& SwarmCommand)
{
    float x,y,z,yaw;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> SwarmCommand <<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

    switch(SwarmCommand.Mode)
    {
        case prometheus_msgs::SwarmCommand::Idle:
            if(SwarmCommand.yaw_ref == 999)
            {
                cout << "Command: [ Idle + Arming + Switching to OFFBOARD mode ] " <<endl;
            }else
            {
                cout << "Command: [ Idle ] " <<endl;
            }
            break;

        case prometheus_msgs::SwarmCommand::Takeoff:
            cout << "Command: [ Takeoff ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Hold:
            cout << "Command: [ Hold ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Land:
            cout << "Command: [ Land ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Disarm:
            cout << "Command: [ Disarm ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Position_Control:
            if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::One_column)
            {
                cout << "Command: [ Position_Control ] [One_column] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Triangle)
            {
                cout << "Command: [ Position_Control ] [Triangle] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Square)
            {
                cout << "Command: [ Position_Control ] [Square] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == prometheus_msgs::SwarmCommand::Circular)
            {
                cout << "Command: [ Position_Control ] [Circular] size: " << SwarmCommand.swarm_size <<endl;
            }
            break;

        case prometheus_msgs::SwarmCommand::Velocity_Control:
            cout << "Command: [ Velocity_Control ] " <<endl;
            break;

        case prometheus_msgs::SwarmCommand::Accel_Control:
            cout << "Command: [ Accel_Control ] " <<endl;

            break;

        case prometheus_msgs::SwarmCommand::Move:

            if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XYZ_POS)
            {
                cout << "Command: [ Move in XYZ_POS] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.position_ref[0] << " [ m ] "<< SwarmCommand.position_ref[1]<<" [ m ] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == prometheus_msgs::SwarmCommand::XY_VEL_Z_POS)
            {
                cout << "Command: [ Move in XY_VEL_Z_POS] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.velocity_ref[0] << " [m/s] "<< SwarmCommand.velocity_ref[1]<<" [m/s] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else
            {
                cout << " wrong sub move mode. " <<endl;
            }
            
            

            break;
    }
}

#endif
