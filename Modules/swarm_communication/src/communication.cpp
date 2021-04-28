#include "communication.h"

int init_broadcast(const char *ip,const int port)
{
    // creating socket file descriptor
    int fd = socket(AF_INET,SOCK_DGRAM,0);
    if(fd <= 0)
    {
        ROS_ERROR("[communication_node]Socket sender creation error!");
        exit(EXIT_FAILURE);
    }

    // attaching socket to the port
    int so_broadcast = 1;
    if(setsockopt(fd,SOL_SOCKET,SO_BROADCAST,&so_broadcast,sizeof(so_broadcast)) < 0)
    {
        printf("Error in setting Broadcast option\n");
        exit(EXIT_FAILURE);
    }
    addr_udp_send.sin_family = AF_INET;
    addr_udp_send.sin_port = htons(port);

    // tf to network address
    if(inet_pton(AF_INET,ip,&addr_udp_send.sin_addr) <= 0)
    {
        printf("Invalid address/ Address not supported\n");
        return -1;
    }

    return fd;
}

int udp_bind_to_port(const int port,int &server_fd)
{
    struct sockaddr_in address;

    // creating socket file descriptor
    server_fd = socket(AF_INET,SOCK_DGRAM,0);
    if(server_fd == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // forcefully attaching socket to the port
    int opt = 1;
    if(setsockopt(server_fd,SOL_SOCKET,SO_REUSEADDR|SO_REUSEPORT,&opt,sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    // bind the port
    if(bind(server_fd, (struct sockaddr *)&address,sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    return server_fd;
}

int encode_drone_state(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    char *ptr = udp_send_buf;

    // drone state length
    int ds_length = sizeof(MESSAGE_TYPE)+sizeof(int8_t)+3*sizeof(float);
    if(ds_length+1 > BUF_LEN)
    {
        ROS_ERROR("[communication_node] Topic is too large, please enlarge BUF_LEN");
        return -1;
    }

    *((MESSAGE_TYPE *)ptr) = MESSAGE_TYPE::ODOM;
    // printf("uav%d encode: %d",uav_id,*((int *)ptr));
    ptr += sizeof(MESSAGE_TYPE);

    *((int8_t *)ptr) = uav_id;
    // printf(" %d",*ptr);
    ptr += sizeof(int8_t);
    
    // printf(" %f",msg->position[0]);
    *((float *)ptr) = msg->position[0];
    ptr += sizeof(float);
    // printf(" %f",msg->position[1]);
    *((float *)ptr) = msg->position[1];
    ptr += sizeof(float);
    // printf(" %f\n",msg->position[2]);
    *((float *)ptr) = msg->position[2];
    ptr += sizeof(float);
    
    return ptr - udp_send_buf;
}

int decode_drone_state(prometheus_msgs::DroneState::Ptr &msg)
{
    char *ptr = udp_recv_buf;

    printf("uav%d decode: %d",uav_id,*((int *)ptr));
    ptr += sizeof(int);

    printf(" %d",*ptr);
    ptr += sizeof(int8_t);

    msg->position[0] = *((float *)ptr);
    printf(" %f",msg->position[0]);
    ptr += sizeof(float);
    msg->position[1] = *((float *)ptr);
    printf(" %f",msg->position[1]);
    ptr += sizeof(float);
    msg->position[2] = *((float *)ptr);
    printf(" %f\n",msg->position[2]);
    ptr += sizeof(float);

    return ptr - udp_recv_buf;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    int len = encode_drone_state(msg);
    if(sendto(udp_send_fd,udp_send_buf,len,0,(struct sockaddr*)&addr_udp_send,sizeof(addr_udp_send)) <= 0)
    {
        ROS_ERROR("TCP SEND ERROR!!!");
    }
    else
    {
        // printf("send:%s\n",udp_send_buf);
    }
}

void udp_recv_fun()
{
    int valread;
    struct sockaddr addr_client;
    socklen_t addr_len;

    // Connect udp: bind port
    if(udp_bind_to_port(UDP_PORT, udp_server_fd) < 0)
    {
        ROS_ERROR("[communication_node]Socket recever creation error!");
        exit(EXIT_FAILURE);
    }
    printf("bind to UDP_PORT\n");

    while(true)
    {
        // receive msg from udp
        valread = recvfrom(udp_server_fd,udp_recv_buf,BUF_LEN,0,&addr_client,&addr_len);
        if(valread < 0)
        {
            perror("recvfrom error:");
            exit(EXIT_FAILURE);
        }

        // decode msg
        char *ptr = udp_recv_buf;
        switch(*((MESSAGE_TYPE *)ptr))
        {
            case MESSAGE_TYPE::ODOM:
            {
                if(valread == decode_drone_state(drone_state_msg))
                {
                    // printf("rec:%s\n",udp_recv_buf);
                    other_drone_state_pub.publish(*drone_state_msg);
                }
                else
                {
                    ROS_ERROR("ODOM:Received message length not matches the sent one!");
                    continue;
                }
                break;
            }

            default:
            {
                ROS_ERROR("Unknown received message");
                break;
            }
        }
        sleep(0.2); // frequence
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ros_topic_2_tcp");
    ros::NodeHandle nh("~");

    // swarm number
    nh.param<int>("swarm_num", swarm_num, 1);
    // uav id and name
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<string>("uav_name", uav_name, "/uav1");
    // broadcast
    nh.param<string>("broadcast_ip", udp_ip, "127.0.0.255");
    nh.param<int>("udp_port", UDP_PORT, 8081);

    drone_state_msg.reset(new prometheus_msgs::DroneState);

    // subscribe
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10, drone_state_cb, ros::TransportHints().tcpNoDelay());
    // publish
    other_drone_state_pub = nh.advertise<prometheus_msgs::DroneState>(uav_name + "/prometheus/ohter_drone_state", 10);

    // thread
    boost::thread udp_recv_thd(udp_recv_fun);
    udp_recv_thd.detach();
    ros::Duration(1).sleep(); // frequence

    // udp connect: init socket
    udp_send_fd = init_broadcast(udp_ip.c_str(), UDP_PORT);

    printf("[communication_node] start running\n");

    ros::spin();

    // close socket
    close(udp_server_fd);
    close(udp_send_fd);

    return 0;
}