//发布Case3Result，Message
#include "ros/ros.h"
#include <boost/format.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "std_msgs/String.h"
#include "prometheus_msgs/Case3Result.h"
#include "prometheus_msgs/StationCommandCase3.h"

#include <sstream>

using namespace std;
bool sim_mode;
bool flag_ros2groundstation;
bool get_result = false;

//参数声明
ros::Subscriber Result_sub;
ros::Publisher case3_cmd_pub;
prometheus_msgs::Case3Result case3_result;
prometheus_msgs::StationCommandCase3 case3_cmd;


char const *servInetAddr = "127.0.0.1"; //sever ip
string data;
char sendline[1024];
int socketfd,connfd;
struct sockaddr_in sockaddr;

//函数声明
void result_callback(const prometheus_msgs::Case3Result::ConstPtr& msg) 
{ 
    case3_result = *msg; 

    if(!get_result)
    {
        // 只发送一次:告诉所有飞机，已经有队友检测到目标了
        get_result = true;
        case3_cmd.Command_uav = prometheus_msgs::StationCommandCase3::Normal;
        case3_cmd.detection_flag = prometheus_msgs::StationCommandCase3::detected;
        case3_cmd.enu_position = case3_result.enu_position;
        case3_cmd_pub.publish(case3_cmd);
    }else
    {
        cout << "get result again!"<< endl;
    }  
}

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "case3_pub_station");
    ros::NodeHandle nh("~");

    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_ros2groundstation", flag_ros2groundstation, false);

    Result_sub = nh.subscribe<prometheus_msgs::Case3Result>("/case3/uav_result", 10, result_callback);

    case3_cmd_pub = nh.advertise<prometheus_msgs::StationCommandCase3>("/case3/command_uav", 1); 
    
    while(ros::ok())
    {   
        ros::spinOnce();
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>> case3_to_groundstation <<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;

        if(get_result)
        {
            cout << "Get Result"<< endl;
            cout << "UAV ID" << case3_result.uav_id << endl;
            cout << "Target Position [X Y Z]" << case3_result.enu_position[0] << " [ m ] "<< case3_result.enu_position[1] << " [ m ] "<< case3_result.enu_position[2] << " [ m ]" << endl;
        }

        // 理论意义上来说，这个也只需要发一次
        // 真实地面站没收到消息，则代表没人检测到目标
        if(flag_ros2groundstation && get_result)
        {
            boost::format fmt3("uav%d,%f,%f,%f");
            printf("send message to server: ");
            data = (fmt3%(case3_result.uav_id)%(case3_result.enu_position[0])%(case3_result.enu_position[1])%(case3_result.enu_position[2])).str();
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
        
        sleep(0.5); // frequence
    }

    return 0;
}