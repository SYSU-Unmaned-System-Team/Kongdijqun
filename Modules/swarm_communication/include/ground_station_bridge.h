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

int send_sock = 0, server_fd, recv_sock, TCP_PORT;
int takeoff_state = 0; //没有起飞
string tcp_ip;
bool debug_mode;
char tcp_send_buf[BUF_LEN], tcp_recv_buf[BUF_LEN], udp_recv_buf[BUF_LEN];
prometheus_msgs::DroneState::Ptr drone_state_msg;
struct sockaddr_in serv_addr;

