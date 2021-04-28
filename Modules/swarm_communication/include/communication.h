#include <ros/ros.h>
#include <string>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <prometheus_msgs/DroneState.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

using namespace std;
#define BUF_LEN 1048576 // 1MB

enum MESSAGE_TYPE { ODOM = 888, MULTI_TRAJ, ONE_TRAJ, STOP } massage_type;

int swarm_num, uav_id, udp_send_fd,udp_server_fd,UDP_PORT;
string uav_name, udp_ip;
struct sockaddr_in addr_udp_send;
char udp_send_buf[BUF_LEN], udp_recv_buf[BUF_LEN];
prometheus_msgs::DroneState::Ptr drone_state_msg;

ros::Subscriber drone_state_sub;
ros::Publisher other_drone_state_pub;