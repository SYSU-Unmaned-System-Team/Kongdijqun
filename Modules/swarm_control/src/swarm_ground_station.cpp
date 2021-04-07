//头文件
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "swarm_ground_station.h"

using namespace std;
//参数声明
int swarm_num;
const int max_swarm_num = 8; // indicate max num
string uav_name[max_swarm_num+1];
int uav_id[max_swarm_num+1];
prometheus_msgs::DroneState State_uav[max_swarm_num+1];
prometheus_msgs::SwarmCommand Command_uav[max_swarm_num+1];
geometry_msgs::PoseStamped ref_pose_uav[max_swarm_num+1];
ros::Subscriber command_sub[max_swarm_num+1];
ros::Subscriber drone_state_sub[max_swarm_num+1];
char *servInetAddr = "127.0.0.1"; //sever ip
string data;
char sendline[1024];
int socketfd;
struct sockaddr_in sockaddr;

//函数声明
void swarm_command_cb_1(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[1] = *msg; }
void swarm_command_cb_2(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[2] = *msg; }
void swarm_command_cb_3(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[3] = *msg; }
void swarm_command_cb_4(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[4] = *msg; }
void swarm_command_cb_5(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[5] = *msg; }
void swarm_command_cb_6(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[6] = *msg; }
void swarm_command_cb_7(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[7] = *msg; }
void swarm_command_cb_8(const prometheus_msgs::SwarmCommand::ConstPtr& msg) { Command_uav[8] = *msg; }
void (*swarm_command_cb[max_swarm_num+1])(const prometheus_msgs::SwarmCommand::ConstPtr&)={NULL,swarm_command_cb_1,
    swarm_command_cb_2,swarm_command_cb_3,swarm_command_cb_4,swarm_command_cb_5,swarm_command_cb_6,swarm_command_cb_7,swarm_command_cb_8};

void drone_state_cb1(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[1] = *msg; }
void drone_state_cb2(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[2] = *msg; }
void drone_state_cb3(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[3] = *msg; }
void drone_state_cb4(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[4] = *msg; }
void drone_state_cb5(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[5] = *msg; }
void drone_state_cb6(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[6] = *msg; }
void drone_state_cb7(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[7] = *msg; }
void drone_state_cb8(const prometheus_msgs::DroneState::ConstPtr& msg) { State_uav[8] = *msg; }
void (*drone_state_cb[max_swarm_num+1])(const prometheus_msgs::DroneState::ConstPtr&)={NULL,drone_state_cb1,drone_state_cb2,
    drone_state_cb3,drone_state_cb4,drone_state_cb5,drone_state_cb6,drone_state_cb7,drone_state_cb8};

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_ground_station");
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num", swarm_num, 1);

    for(int i = 1; i <= swarm_num; i++) 
    {
        // 设置无人机名字，none代表无
        boost::format fmt1("uav%d_name");
        nh.param<string>((fmt1%(i)).str(), uav_name[i], "/none");
        boost::format fmt2("uav%d_id");
        nh.param<int>((fmt2%(i)).str(), uav_id[i], 0);
        // 订阅
        command_sub[i] = nh.subscribe<prometheus_msgs::SwarmCommand>(uav_name[i] + "/prometheus/swarm_command", 10, swarm_command_cb[i]);
        drone_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>(uav_name[i] + "/prometheus/drone_state", 10, drone_state_cb[i]);
    }
    
    boost::format fmt3("uav%d,%f,%f,%f,%f,%f,%f,%f,%f,%f");
    while(ros::ok())
    {
        ros::spinOnce();
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>> Formation Flight Station <<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        for(int i = 1; i <= swarm_num; i++)
        {
            if(uav_id[i] != 0)
            {
                printf_swarm_state(swarm_num, uav_id[i], uav_name[i], State_uav[i], Command_uav[i]);
            }

            printf("send message to server: ");
            data = (fmt3%(i)%(State_uav[i].position[0])%(State_uav[i].position[1])%State_uav[i].position[2]%
                (State_uav[i].velocity[0])%(State_uav[i].velocity[1])%(State_uav[i].velocity[2])%
                (State_uav[i].attitude[0])%(State_uav[i].attitude[1])%(State_uav[i].attitude[2])).str();
            cout << data << endl;
            strcpy(sendline,data.c_str());
            //send by socket
            socketfd = socket(AF_INET,SOCK_STREAM,0);
            memset(&sockaddr,0,sizeof(sockaddr));
            sockaddr.sin_family = AF_INET;
            sockaddr.sin_port = htons(10004);
            inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);
            if((connect(socketfd,(struct sockaddr*)&sockaddr,sizeof(sockaddr))) < 0 ) {
                printf("connect error %s errno: %d\n",strerror(errno),errno);
                printf("client connect failed!\n");
            }
            if((send(socketfd,sendline,strlen(sendline),0)) < 0)
            {
                printf("send mes error: %s errno : %d\n",strerror(errno),errno);
                printf("client send failed!\n");
            }
            close(socketfd);

        }
        sleep(2.0); // frequence
    }
    return 0;
}