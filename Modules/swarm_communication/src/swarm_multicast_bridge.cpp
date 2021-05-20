#include "ground_station_bridge.h"
#include "swarm_formation_control.h"

int connect_to_ground_station(const char *ip, const int port)
{
    // connect
    memset(&serv_addr,0,sizeof(serv_addr));
    if(send_sock = socket(AF_INET,SOCK_DGRAM,0) < 0)
    {
        printf("\n Socket creation error \n");
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = inet_addr(ip);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET,ip,&serv_addr.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    return send_sock;
}

int encode_drone_state(const prometheus_msgs::DroneState::ConstPtr& msg, int uav_id_)
{
    char *ptr = tcp_send_buf;

    // drone state length
    int ds_length = sizeof(int8_t)+3*3*sizeof(float);
    if(ds_length+1 > BUF_LEN)
    {
        ROS_ERROR("[communication_node] Topic is too large, please enlarge BUF_LEN");
        return -1;
    }

    *((int8_t *)ptr) = uav_id_; //id
    ptr += sizeof(int8_t);
    
    *((float *)ptr) = msg->position[0]; //xyz
    ptr += sizeof(float);
    *((float *)ptr) = msg->position[1];
    ptr += sizeof(float);
    *((float *)ptr) = msg->position[2];
    ptr += sizeof(float);

    *((float *)ptr) = msg->velocity[0]; //velocity
    ptr += sizeof(float);
    *((float *)ptr) = msg->velocity[1];
    ptr += sizeof(float);
    *((float *)ptr) = msg->velocity[2];
    ptr += sizeof(float);

    *((float *)ptr) = msg->attitude[0]; //attitude
    ptr += sizeof(float);
    *((float *)ptr) = msg->attitude[1];
    ptr += sizeof(float);
    *((float *)ptr) = msg->attitude[2];
    ptr += sizeof(float);
    
    return ptr - tcp_send_buf; // msg length
}

int wait_connection_from_ground_station(const int port)
{
    struct sockaddr_in address;
    int opt = 1;
    // int addrlen = sizeof(address);
    memset(&address,0,sizeof(address));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(TCP_PORT);

    // Creating socket file descriptor
    if((server_fd = socket(AF_INET,SOCK_STREAM,0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // set socket
    if (setsockopt(server_fd,SOL_SOCKET,SO_REUSEADDR|SO_REUSEPORT,&opt,sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port
    if(bind(server_fd,(struct sockaddr *)&address,sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if(listen(server_fd, BUF_LEN) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    return recv_sock;
}

void server_fun()
{
    int valread;
    // udp connect
    if(wait_connection_from_ground_station(TCP_PORT) < 0)
    {
        ROS_ERROR("[bridge_node]Socket recever creation error!");
        exit(EXIT_FAILURE);
    }

    while(true)
    {
        if((recv_sock = accept(server_fd,(struct sockaddr *)NULL,NULL) )< 0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }
        valread = recv(recv_sock,udp_recv_buf,BUF_LEN,0);

        if(valread <= 0)
        {
            ROS_ERROR("Received message length <= 0, maybe connection has lost");
            close(recv_sock);
            close(server_fd);
            return;
        }
        // takeoff
        if((udp_recv_buf[0] == '9') && !takeoff_state)
        {
            printf("All the UAVs are disarmed and takeoff.\n");
            takeoff_state = 1;
            disarm_swarm();
            sleep(2);
            takeoff_swarm();
        }
        // swarm formation
        if(takeoff_state&&(udp_recv_buf[0] == '1')||(udp_recv_buf[0] == '2')||(udp_recv_buf[0] == '3')||(udp_recv_buf[0] == '4'))
        {
            formation_num = udp_recv_buf[0]-'0';
            pub_formation_command();
        }
        // land
        if((udp_recv_buf[0] == '8') && (takeoff_state == 1))
        {
            printf("All the UAVs are armed and land.\n");
            takeoff_state = 0;
            land_swarm();
            sleep(8);
            arm_swarm();
        }
        // reset buffer
        memset(udp_recv_buf,0,sizeof(udp_recv_buf));
    }
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg,int& uav_id_)
{
    int len = encode_drone_state(msg,uav_id_);
    sendto(send_sock,tcp_send_buf,len,0,(struct sockaddr*)&serv_addr,sizeof(serv_addr));
}

int main(int argc, char **argv) 
{   

    ros::init(argc,argv,"swarm_multicast_bridge");
    ros::NodeHandle nh("~");

    // swarm
    nh.param<int>("swarm_num",swarm_num,1);
    nh.param<float>("formation_size",formation_size,1.0);
    // tcp/udp
    nh.param<string>("ground_station_ip",tcp_ip,"192.168.1.21");
    nh.param<int>("tcp_port",TCP_PORT,8887);
    // swarm control
    nh.param<int>("controller_num",controller_num,0);
    // virtual leader
    virtual_leader_pos << 0.0,0.0,1.0;
    virtual_leader_yaw = 0.0;

    drone_state_msg.reset(new prometheus_msgs::DroneState);

    for(int i = 1; i <= swarm_num; i++) 
    {
        // uav name
        boost::format fmt1("uav%d_name");
        nh.param<string>((fmt1%(i)).str(), uav_name[i], "/none");
        // uav id
        boost::format fmt2("uav%d_id");
        nh.param<int>((fmt2%(i)).str(), uav_id[i], 0);
        // subscribe
        drone_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/"+uav_name[i]+"/prometheus/drone_state",10,boost::bind(&drone_state_cb,_1,uav_id[i]));
        // publish
        command_pub[i] = nh.advertise<prometheus_msgs::SwarmCommand>("/"+uav_name[i]+"/prometheus/swarm_command",10); //【发布】阵型    
    }

    // thread
    boost::thread recv_thd(server_fun);
    recv_thd.detach();
    ros::Duration(1).sleep(); // wait

    // tcp connect: init socket
    send_sock = connect_to_ground_station(tcp_ip.c_str(),TCP_PORT);

    printf("[swarm_multicast_bridge_node] start running\n");
    ros::spin();

    // close socket
    close(send_sock);
    close(recv_sock);
    close(server_fd);
    return 0;
}