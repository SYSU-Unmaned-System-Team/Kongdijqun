//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

#define NODE_NAME "swarm_formation_control"
#define MAX_NUM 40
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int swarm_num;
string uav_name[MAX_NUM];
int uav_id[MAX_NUM];
prometheus_msgs::SwarmCommand swarm_command[MAX_NUM];
ros::Publisher command_pub[MAX_NUM];

int controller_num;
float formation_size;

Eigen::Vector3f virtual_leader_pos;
Eigen::Vector3f virtual_leader_vel;
float virtual_leader_yaw;

int formation_num = 1;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();
void pub_formation_command();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    cout << "Get a new goal from rviz!"<<endl;
    virtual_leader_pos[0] = msg->pose.position.x;
    virtual_leader_pos[1] = msg->pose.position.y;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    
    nh.param<int>("swarm_num", swarm_num, 1);
    // 0代表位置追踪模式，１代表速度追踪模式，２代表加速度追踪模式 
    nh.param<int>("controller_num", controller_num, 0);
    nh.param<float>("formation_size", formation_size, 1.0);

    virtual_leader_pos << 0.0,0.0,1.0;
    virtual_leader_yaw = 0.0;

    //【订阅】目标点
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/formation/virtual_leader", 10, goal_cb);

    for(int i = 1; i <= swarm_num; i++) 
    {
        // 设置无人机名字，none代表无
        boost::format fmt1("uav%d_name");
        nh.param<string>((fmt1%(i)).str(), uav_name[i], "/none");

        boost::format fmt2("uav%d_id");
        nh.param<int>((fmt2%(i)).str(), uav_id[i], 0);

        if(uav_name[i] == "/none" || uav_id[i] == 0)
        {
            cout << "Wrong UAV Name or ID!"<< endl;
        }

        //【发布】阵型
        command_pub[i] = nh.advertise<prometheus_msgs::SwarmCommand>(uav_name[i] + "/prometheus/swarm_command", 10);
    }

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

    // Waiting for input
    int start_flag = 0;

    printf_param();

    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to disarm all the UAVs."<<endl;
        cin >> start_flag;

        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Idle;
            swarm_command[i].yaw_ref = 999;
            //【发布】阵型
            command_pub[i].publish(swarm_command[i]);
        }
    }

    start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff all the UAVs."<<endl;
        cin >> start_flag;

        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Takeoff;
            swarm_command[i].yaw_ref = 0.0;
            //【发布】阵型
            command_pub[i].publish(swarm_command[i]);
        }
    }
    
    float trajectory_total_time;
    while (ros::ok()) // todo: only check start_flag=0, other function need tested
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< 3 for Circle Trajectory Tracking,"<< endl;
        cout << "Please choose the action: 0 for Formation Shape, 1 for Virtual Leader Pos, 2 for Hold, 3 for Land, 4 for Circle, 5 for Disarm..."<<endl;
        cin >> start_flag;
        if (start_flag == 0)
        {
            cout << "Please choose the formation: 1 for One Column, 2 for Triangle, 3 for Square, 4 for Circular ..."<<endl;
            cin >> formation_num;

            pub_formation_command();
            
        }
        else if (start_flag == 1)
        {
            cout << "Please enter the virtual leader position:"<<endl;
            cout << "virtual_leader_pos: --- x [m] "<<endl;
            cin >> virtual_leader_pos[0];
            cout << "virtual_leader_pos: --- y [m]"<<endl;
            cin >> virtual_leader_pos[1];
            cout << "virtual_leader_pos: --- z [m]"<<endl;
            cin >> virtual_leader_pos[2];
            cout << "virtual_leader_yaw [deg]:"<<endl;
            cin >> virtual_leader_yaw;
            virtual_leader_yaw = virtual_leader_yaw/180.0*M_PI;

            pub_formation_command();
            
        }
        else if (start_flag == 2)
        {
            for(int i = 1; i <= swarm_num; i++) 
            {
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Hold;
                swarm_command[i].yaw_ref = 999;
                //【发布】阵型
                command_pub[i].publish(swarm_command[i]);
            }
        }
        else if (start_flag == 3)
        {
            for(int i = 1; i <= swarm_num; i++) 
            {
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Land;
                swarm_command[i].yaw_ref = 999;
                //【发布】阵型
                command_pub[i].publish(swarm_command[i]);
            }
        }
        else if (start_flag == 4)
        {
            cout << "Input the trajectory_total_time:"<<endl;
            cin >> trajectory_total_time;

            float time_trajectory = 0.0;

            while(time_trajectory < trajectory_total_time)
            {

                const float omega = 0.15;
                const float circle_radius = 1.5;

                virtual_leader_pos[0] = circle_radius * cos(time_trajectory * omega);
                virtual_leader_pos[1] = circle_radius * sin(time_trajectory * omega);
                //virtual_leader_pos[2] = 1.0;
                virtual_leader_vel[0] = - omega * circle_radius * sin(time_trajectory * omega);
                virtual_leader_vel[1] = omega * circle_radius * cos(time_trajectory * omega);
                virtual_leader_vel[2] = 0.0;

                time_trajectory = time_trajectory + 0.01;

                ros::Duration(0.01).sleep();
            }
        }
        else if (start_flag == 5)
        {
            for(int i = 1; i <= swarm_num; i++) 
            {
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Disarm;
                swarm_command[i].yaw_ref = 999;
                //【发布】阵型
                command_pub[i].publish(swarm_command[i]);
            }
        }
        else
        {
            cout << "Wrong input."<<endl;
        }
        
        cout << "virtual_leader_pos [X Y] : " << virtual_leader_pos[0] << " [ m ] "<< virtual_leader_pos[1] <<" [ m ] "<< virtual_leader_pos[2] <<" [ m ] "<< endl;
        cout << "virtual_leader_yaw: " << virtual_leader_yaw/M_PI*180.0 <<" [ deg ] "<< endl;
     
        ros::Duration(1.0).sleep();
    }
    return 0;
}

void pub_formation_command()
{
    if(formation_num == 1)
    {
        cout << "Formation shape: [ One_column ]"<<endl;
        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].swarm_shape = prometheus_msgs::SwarmCommand::One_column;
        }
    }
    else if(formation_num == 2)
    {
        cout << "Formation shape: [ Triangle ]"<<endl;
        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].swarm_shape = prometheus_msgs::SwarmCommand::Triangle;
        }
    }
    else if(formation_num == 3)
    {
        cout << "Formation shape: [ Square ]"<<endl;
        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].swarm_shape = prometheus_msgs::SwarmCommand::Square;
        }
    }
    else if(formation_num == 4)
    {
        cout << "Formation shape: [ Circular ]"<<endl;
        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].swarm_shape = prometheus_msgs::SwarmCommand::Circular;
        }
    }
    else
    {
        cout << "Wrong formation shape!"<<endl;
    }

    if(controller_num == 0)
    {
        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Position_Control;
        }
    }
    else if(controller_num == 1)
    {
        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Velocity_Control;
        }
    }
    else if(controller_num == 2)
    {
        for(int i = 1; i <= swarm_num; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Accel_Control;
        }
    }

    // cout << "controller_num: " << controller_num << endl;


    for(int i = 1; i <= swarm_num; i++) 
    {
        swarm_command[i].swarm_size = formation_size;
        swarm_command[i].position_ref[0] = virtual_leader_pos[0] ; 
        swarm_command[i].position_ref[1] = virtual_leader_pos[1] ;
        swarm_command[i].position_ref[2] = virtual_leader_pos[2] ;  
        swarm_command[i].velocity_ref[0] = virtual_leader_vel[0] ; 
        swarm_command[i].velocity_ref[1] = virtual_leader_vel[1] ; 
        swarm_command[i].velocity_ref[2] = virtual_leader_vel[2] ; 
        swarm_command[i].yaw_ref = virtual_leader_yaw;
        command_pub[i].publish(swarm_command[i]);
    }
    
    cout << "virtual_leader_pos: " << virtual_leader_pos[0] << "m "<<  virtual_leader_pos[1] << "m "<<  virtual_leader_pos[2] << "m "<< endl;
}

void printf_param()
{
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