// #include "ground_station_bridge.h"
// #include "swarm_formation_control.h"

// int take_state = 0; //没有起飞

// int  connect_to_ground_station(const char *ip, const int port)
// {
//     /* Connect */
//     int sock = 0; 
//     cout<< ip <<endl;
//     struct sockaddr_in serv_addr;

//     if((sock = socket(AF_INET,SOCK_STREAM,0)) < 0)
//     {
//         printf("\n Socket creation error \n");
//         return -1;
//     }
//     memset(&serv_addr,0,sizeof(serv_addr));
//     serv_addr.sin_family = AF_INET;
//     serv_addr.sin_port = htons(port);

//     // Convert IPv4 and IPv6 addresses from text to binary form
//     if(inet_pton(AF_INET,ip,&serv_addr.sin_addr) <= 0)
//     { 
//         printf("\nInvalid address/ Address not supported \n");
//         return -1;
//     }
//     if(connect(sock,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0)
//     {
//         ROS_WARN("Tcp connection to drone_%d Failed");
//         return -1;
//     }
//     char str[INET_ADDRSTRLEN];
//     ROS_INFO("Connect to %s success!", inet_ntop(AF_INET, &serv_addr.sin_addr, str, sizeof(str)));

//     return sock;
// }

// int encode_drone_state(const prometheus_msgs::DroneState::ConstPtr& msg,int uav_id_)
// {
//     char *ptr = tcp_send_buf;

//     // drone state length
//     int ds_length = sizeof(int8_t)+3*3*sizeof(float);
//     if(ds_length+1 > BUF_LEN)
//     {
//         ROS_ERROR("[communication_node] Topic is too large, please enlarge BUF_LEN");
//         return -1;
//     }

//     // printf("uav%d encode: ",uav_id_);

//     *((int8_t *)ptr) = uav_id_;
//     // // printf(" %d",*ptr);
//     ptr += sizeof(int8_t);
    
//     //printf(" %f",msg->position[0]);
//     *((float *)ptr) = msg->position[0];
//     ptr += sizeof(float);
//     // printf(" %f",msg->position[1]);
//     *((float *)ptr) = msg->position[1];
//     ptr += sizeof(float);
//     // printf(" %f\n",msg->position[2]);
//     *((float *)ptr) = msg->position[2];
//     ptr += sizeof(float);

//     // printf(" %f",msg->velocity[0]);
//     *((float *)ptr) = msg->velocity[0];
//     ptr += sizeof(float);
//     // printf(" %f",msg->velocity[1]);
//     *((float *)ptr) = msg->velocity[1];
//     ptr += sizeof(float);
//     // printf(" %f\n",msg->velocity[2]);
//     *((float *)ptr) = msg->velocity[2];
//     ptr += sizeof(float);

//     // printf(" %f",msg->attitude[0]);
//     *((float *)ptr) = msg->attitude[0];
//     ptr += sizeof(float);
//     // printf(" %f",msg->attitude[1]);
//     *((float *)ptr) = msg->attitude[1];
//     ptr += sizeof(float);
//     // printf(" %f\n",msg->attitude[2]);
//     *((float *)ptr) = msg->attitude[2];
//     ptr += sizeof(float);
    
//     return ptr - tcp_send_buf;
// }

// int decode_drone_state(prometheus_msgs::DroneState::Ptr &msg,int uav_id_)
// {
//     char *ptr = tcp_recv_buf;

//     printf("uav%d decode: %d",uav_id_,*((int *)ptr));
//     ptr += sizeof(int);

//     printf(" %d",*ptr);
//     ptr += sizeof(int8_t);

//     msg->position[0] = *((float *)ptr);
//     printf(" %f",msg->position[0]);
//     ptr += sizeof(float);
//     msg->position[1] = *((float *)ptr);
//     printf(" %f",msg->position[1]);
//     ptr += sizeof(float);
//     msg->position[2] = *((float *)ptr);
//     printf(" %f\n",msg->position[2]);
//     ptr += sizeof(float);

//     msg->velocity[0] = *((float *)ptr);
//     printf(" %f",msg->velocity[0]);
//     ptr += sizeof(float);
//     msg->velocity[1] = *((float *)ptr);
//     printf(" %f",msg->velocity[1]);
//     ptr += sizeof(float);
//     msg->velocity[2] = *((float *)ptr);
//     printf(" %f\n",msg->velocity[2]);
//     ptr += sizeof(float);

//     msg->attitude[0] = *((float *)ptr);
//     printf(" %f",msg->attitude[0]);
//     ptr += sizeof(float);
//     msg->attitude[1] = *((float *)ptr);
//     printf(" %f",msg->attitude[1]);
//     ptr += sizeof(float);
//     msg->attitude[2] = *((float *)ptr);
//     printf(" %f\n",msg->attitude[2]);
//     ptr += sizeof(float);

//     return ptr - tcp_recv_buf;
// }

// int wait_connection_from_ground_station(const int port)
// {
//     struct sockaddr_in address;
//     int opt = 1;
//     // int addrlen = sizeof(address);
//     memset(&address,0,sizeof(address));

//     address.sin_family = AF_INET;
//     address.sin_addr.s_addr = htonl(INADDR_ANY);
//     address.sin_port = htons(TCP_PORT);

//     // Creating socket file descriptor
//     if((server_fd = socket(AF_INET,SOCK_STREAM,0)) == 0)
//     {
//         perror("socket failed");
//         exit(EXIT_FAILURE);
//     }

//     // Forcefully attaching socket to the port
//     if (setsockopt(server_fd,SOL_SOCKET,SO_REUSEADDR|SO_REUSEPORT,&opt,sizeof(opt)))
//     {
//         perror("setsockopt");
//         exit(EXIT_FAILURE);
//     }
//     // Forcefully attaching socket to the port

//     if(bind(server_fd,(struct sockaddr *)&address,sizeof(address)) < 0)
//     {
//         perror("bind failed");
//         exit(EXIT_FAILURE);
//     }
//     if(listen(server_fd, BUF_LEN) < 0)
//     {
//         perror("listen");
//         exit(EXIT_FAILURE);
//     }

//     return recv_sock;
//     // char str[INET_ADDRSTRLEN];
//     // ROS_INFO( "Receive tcp connection from %s", inet_ntop(AF_INET, &address.sin_addr, str, sizeof(str)) );
// }

// void server_fun()
// {
//     int valread;
//     // Connect
//     if(wait_connection_from_ground_station(TCP_PORT) < 0)
//     {
//         ROS_ERROR("[bridge_node]Socket recever creation error!");
//         exit(EXIT_FAILURE);
//     }

//     while(true)
//     {
//         if((recv_sock = accept(server_fd,(struct sockaddr *)NULL,NULL) )< 0)
//         {
//             perror("accept");
//             exit(EXIT_FAILURE);
//         }
//         valread = recv(recv_sock, tcp_recv_buf, BUF_LEN ,0);
//         tcp_recv_buf[valread] = '\0';

//         // cout<<tcp_recv_buf[0]<<endl;

//         if(valread <= 0)
//         {
//             ROS_ERROR("Received message length <= 0, maybe connection has lost");
//             close(recv_sock);
//             close(server_fd);
//             return;
//         }

//         cout<<tcp_recv_buf[0]<<"    "<<take_state<<endl;

//         if((tcp_recv_buf[0] == '9') && (take_state == 0))
//         {
//             cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
//             cout << "All the UAVs are disarmed and takeoff."<<endl;
//             take_state = 1;
//             // sleep(2);
//             disarm_swarm();
//             sleep(2);
//             takeoff_swarm();
//         }

//         if((take_state == 1)&&(tcp_recv_buf[0] == '1')||(tcp_recv_buf[0] == '2')||(tcp_recv_buf[0] == '3')||(tcp_recv_buf[0] == '4'))
//         {
//             formation_num = tcp_recv_buf[0]-'0';
//             pub_formation_command();
//         }
        
//         if((tcp_recv_buf[0] == '8') && (take_state == 1))
//         {
//             cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
//             cout << "All the UAVs are armed and land."<<endl;
//             // sleep(2);
//             take_state = 0;
//             land_swarm();
//             sleep(8);
//             arm_swarm();
//             virtual_leader_pos << 0.0,0.0,1.0;
//             virtual_leader_yaw = 0.0;
//         }

//         memset(tcp_recv_buf,0,sizeof(tcp_recv_buf));
//     }

//     close(recv_sock);
//     close(server_fd);

// }

// void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg,int& uav_id_)
// {
//     int len = encode_drone_state(msg, uav_id_);
//     send_sock = connect_to_ground_station(tcp_ip.c_str(), TCP_PORT);
//     if(send(send_sock,tcp_send_buf,len,0) <= 0)
//     {
//         ROS_ERROR("TCP SEND ERROR!!!");
//     }
//     else
//     {
//         printf("send:%s\n",tcp_send_buf);
//     }
//     close(send_sock);
// }

// int main(int argc, char **argv) 
// {   

//     ros::init(argc, argv, "ground_station_bridge");
//     ros::NodeHandle nh("~");

//     // swarm
//     nh.param<int>("swarm_num", swarm_num, 1);
//     nh.param<float>("formation_size", formation_size, 1.0);
//     // broadcast
//     nh.param<string>("ground_station_ip", tcp_ip, "192.168.1.21");
//     nh.param<int>("tcp_port", TCP_PORT, 10004);
//     // swarm control
//     nh.param<int>("controller_num", controller_num, 0);

//     virtual_leader_pos << 0.0,0.0,1.0;
//     virtual_leader_yaw = 0.0;

//     drone_state_msg.reset(new prometheus_msgs::DroneState);

//     for(int i = 1; i <= swarm_num; i++) 
//     {
//         // uav id and name
//         boost::format fmt1("uav%d");
//         uav_name[i]= (fmt1%(i)).str();  
//         uav_id[i]=i;
//         // subscribe
//         drone_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/"+uav_name[i] + "/prometheus/drone_state", 10, boost::bind(&drone_state_cb,_1,uav_id[i]));
//         // publish
//         command_pub[i] = nh.advertise<prometheus_msgs::SwarmCommand>("/"+uav_name[i] + "/prometheus/swarm_command", 10); //【发布】阵型    
//     }

//     // thread
//     boost::thread recv_thd(server_fun);
//     recv_thd.detach();
//     ros::Duration(1).sleep(); // frequence

//     // tcp connect: init socket
//     // send_sock = connect_to_ground_station(tcp_ip.c_str(), TCP_PORT);

//     // printf("[%s ground_station_bridge_node] start running\n",uav_name_);
//     printf("start running\n");
//     ros::spin();

//     // close socket
//     printf("closing sock\n");
//     // close(send_sock);

//     // cout<<"hello"<<endl;

//     return 0;
// }

// #include "ground_station_bridge.h"
// #include "swarm_formation_control.h"

// int take_state = 0; //没有起飞
// struct sockaddr_in serv_addr;

// int connect_to_ground_station(const char *ip, const int port)
// {
//     /* Connect */
//     //  int send_sock = 0; 
//     cout << "Prepare to send by UDP" << endl;
//     cout << ip << endl; 
    
//     memset(&serv_addr,0,sizeof(serv_addr));
//     int send_sock = socket(AF_INET,SOCK_DGRAM,0);
//     if(send_sock < 0) //
//     {
//         printf("\n Socket creation error \n");
//     }
//     serv_addr.sin_family = AF_INET;
//     serv_addr.sin_port = htons(port);
//     serv_addr.sin_addr.s_addr = inet_addr(ip);
//     // serv_addr.sin_addr.s_addr = INADDR_ANY;

//     // Convert IPv4 and IPv6 addresses from text to binary form
//     if(inet_pton(AF_INET,ip,&serv_addr.sin_addr) <= 0)
//     {
//         printf("\nInvalid address/ Address not supported \n");
//         return -1;
//     }

//     return send_sock;

// }

// int encode_drone_state(const prometheus_msgs::DroneState::ConstPtr& msg,int uav_id_)
// {
//     char *ptr = tcp_send_buf;

//     // drone state length
//     int ds_length = sizeof(int8_t)+3*3*sizeof(float);
//     if(ds_length+1 > BUF_LEN)
//     {
//         ROS_ERROR("[communication_node] Topic is too large, please enlarge BUF_LEN");
//         return -1;
//     }

//     // printf("uav%d encode: ",uav_id_);

//     *((int8_t *)ptr) = uav_id_;
//     // // printf(" %d",*ptr);
//     ptr += sizeof(int8_t);
    
//     //printf(" %f",msg->position[0]);
//     *((float *)ptr) = msg->position[0];
//     ptr += sizeof(float);
//     // printf(" %f",msg->position[1]);
//     *((float *)ptr) = msg->position[1];
//     ptr += sizeof(float);
//     // printf(" %f\n",msg->position[2]);
//     *((float *)ptr) = msg->position[2];
//     ptr += sizeof(float);

//     // printf(" %f",msg->velocity[0]);
//     *((float *)ptr) = msg->velocity[0];
//     ptr += sizeof(float);
//     // printf(" %f",msg->velocity[1]);
//     *((float *)ptr) = msg->velocity[1];
//     ptr += sizeof(float);
//     // printf(" %f\n",msg->velocity[2]);
//     *((float *)ptr) = msg->velocity[2];
//     ptr += sizeof(float);

//     // printf(" %f",msg->attitude[0]);
//     *((float *)ptr) = msg->attitude[0];
//     ptr += sizeof(float);
//     // printf(" %f",msg->attitude[1]);
//     *((float *)ptr) = msg->attitude[1];
//     ptr += sizeof(float);
//     // printf(" %f\n",msg->attitude[2]);
//     *((float *)ptr) = msg->attitude[2];
//     ptr += sizeof(float);
    
//     return ptr - tcp_send_buf;
// }

// int decode_drone_state(prometheus_msgs::DroneState::Ptr &msg,int uav_id_)
// {
//     char *ptr = tcp_recv_buf;

//     printf("uav%d decode: %d",uav_id_,*((int *)ptr));
//     ptr += sizeof(int);

//     printf(" %d",*ptr);
//     ptr += sizeof(int8_t);

//     msg->position[0] = *((float *)ptr);
//     printf(" %f",msg->position[0]);
//     ptr += sizeof(float);
//     msg->position[1] = *((float *)ptr);
//     printf(" %f",msg->position[1]);
//     ptr += sizeof(float);
//     msg->position[2] = *((float *)ptr);
//     printf(" %f\n",msg->position[2]);
//     ptr += sizeof(float);

//     msg->velocity[0] = *((float *)ptr);
//     printf(" %f",msg->velocity[0]);
//     ptr += sizeof(float);
//     msg->velocity[1] = *((float *)ptr);
//     printf(" %f",msg->velocity[1]);
//     ptr += sizeof(float);
//     msg->velocity[2] = *((float *)ptr);
//     printf(" %f\n",msg->velocity[2]);
//     ptr += sizeof(float);

//     msg->attitude[0] = *((float *)ptr);
//     printf(" %f",msg->attitude[0]);
//     ptr += sizeof(float);
//     msg->attitude[1] = *((float *)ptr);
//     printf(" %f",msg->attitude[1]);
//     ptr += sizeof(float);
//     msg->attitude[2] = *((float *)ptr);
//     printf(" %f\n",msg->attitude[2]);
//     ptr += sizeof(float);

//     return ptr - tcp_recv_buf;
// }

// int wait_connection_from_ground_station(const int port)
// {
//     struct sockaddr_in address;
//     int opt = 1;
//     // int addrlen = sizeof(address);
//     memset(&address,0,sizeof(address));

//     address.sin_family = AF_INET;
//     address.sin_addr.s_addr = htonl(INADDR_ANY);
//     address.sin_port = htons(TCP_PORT);

//     // Creating socket file descriptor
//     if((server_fd = socket(AF_INET,SOCK_STREAM,0)) == 0)
//     {
//         perror("socket failed");
//         exit(EXIT_FAILURE);
//     }

//     // Forcefully attaching socket to the port
//     if (setsockopt(server_fd,SOL_SOCKET,SO_REUSEADDR|SO_REUSEPORT,&opt,sizeof(opt)))
//     {
//         perror("setsockopt");
//         exit(EXIT_FAILURE);
//     }
//     // Forcefully attaching socket to the port

//     if(bind(server_fd,(struct sockaddr *)&address,sizeof(address)) < 0)
//     {
//         perror("bind failed");
//         exit(EXIT_FAILURE);
//     }
//     if(listen(server_fd, BUF_LEN) < 0)
//     {
//         perror("listen");
//         exit(EXIT_FAILURE);
//     }

//     return recv_sock;
//     // char str[INET_ADDRSTRLEN];
//     // ROS_INFO( "Receive tcp connection from %s", inet_ntop(AF_INET, &address.sin_addr, str, sizeof(str)) );
// }

// void server_fun()
// {
//     int valread;
//     // Connect
//     if(wait_connection_from_ground_station(TCP_PORT) < 0)
//     {
//         ROS_ERROR("[bridge_node]Socket recever creation error!");
//         exit(EXIT_FAILURE);
//     }

//     while(true)
//     {
//         if((recv_sock = accept(server_fd,(struct sockaddr *)NULL,NULL) )< 0)
//         {
//             perror("accept");
//             exit(EXIT_FAILURE);
//         }
//         valread = recv(recv_sock, tcp_recv_buf, BUF_LEN ,0);
//         tcp_recv_buf[valread] = '\0';

//         // cout<<tcp_recv_buf[0]<<endl;

//         if(valread <= 0)
//         {
//             ROS_ERROR("Received message length <= 0, maybe connection has lost");
//             close(recv_sock);
//             close(server_fd);
//             return;
//         }

//         cout<<tcp_recv_buf[0]<<"    "<<take_state<<endl;

//         if((tcp_recv_buf[0] == '9') && (take_state == 0))
//         {
//             cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
//             cout << "All the UAVs are disarmed and takeoff."<<endl;
//             take_state = 1;
//             // sleep(2);
//             disarm_swarm();
//             sleep(2);
//             takeoff_swarm();
//         }

//         if((take_state == 1)&&(tcp_recv_buf[0] == '1')||(tcp_recv_buf[0] == '2')||(tcp_recv_buf[0] == '3')||(tcp_recv_buf[0] == '4'))
//         {
//             formation_num = tcp_recv_buf[0]-'0';
//             pub_formation_command();
//         }
        
//         if((tcp_recv_buf[0] == '8') && (take_state == 1))
//         {
//             cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
//             cout << "All the UAVs are armed and land."<<endl;
//             // sleep(2);
//             take_state = 0;
//             land_swarm();
//             sleep(8);
//             arm_swarm();
//             virtual_leader_pos << 0.0,0.0,1.0;
//             virtual_leader_yaw = 0.0;
//         }

//         memset(tcp_recv_buf,0,sizeof(tcp_recv_buf));
//     }

//     close(recv_sock);
//     close(server_fd);

// }

// void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg,int& uav_id_)
// {
//     int len = encode_drone_state(msg,uav_id_);
//     sendto(send_sock,tcp_send_buf,len,0,(struct sockaddr*)&serv_addr,sizeof(serv_addr));
//     // close(send_sock); 
// }

// int main(int argc, char **argv) 
// {   

//     ros::init(argc, argv, "ground_station_bridge");
//     ros::NodeHandle nh("~");

//     // swarm
//     nh.param<int>("swarm_num", swarm_num, 1);
//     nh.param<float>("formation_size", formation_size, 1.0);
//     // broadcast
//     nh.param<string>("ground_station_ip", tcp_ip, "192.168.1.21");
//     nh.param<int>("tcp_port", TCP_PORT, 8887);
//     // swarm control
//     nh.param<int>("controller_num", controller_num, 0);

//     virtual_leader_pos << 0.0,0.0,1.0;
//     virtual_leader_yaw = 0.0;

//     drone_state_msg.reset(new prometheus_msgs::DroneState);

//     for(int i = 1; i <= swarm_num; i++) 
//     {
//         // uav id and name
//         boost::format fmt1("uav%d");
//         uav_name[i]= (fmt1%(i)).str();  
//         uav_id[i]=i;
//         // subscribe
//         drone_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/"+uav_name[i] + "/prometheus/drone_state", 10, boost::bind(&drone_state_cb,_1,uav_id[i]));
//         // publish
//         command_pub[i] = nh.advertise<prometheus_msgs::SwarmCommand>("/"+uav_name[i] + "/prometheus/swarm_command", 10); //【发布】阵型    
//     }

//     // thread
//     boost::thread recv_thd(server_fun);
//     recv_thd.detach();
//     ros::Duration(1).sleep(); // frequence

//     // tcp connect: init socket

//     connect_to_ground_station(tcp_ip.c_str(), TCP_PORT);

//     printf("start running\n");
//     ros::spin();

//     // close socket
//     printf("closing sock\n");
//     close(send_sock);


//     // cout<<"hello"<<endl;

//     return 0;
// }


#include "ground_station_bridge.h"
#include "swarm_formation_control.h"

int take_state = 0; //没有起飞
struct sockaddr_in serv_addr;

int connect_to_ground_station(const char *ip, const int port)
{
    /* Connect */
    //  int send_sock = 0; 
    cout << "Prepare to send by UDP" << endl;
    cout << ip << endl; 
    
    memset(&serv_addr,0,sizeof(serv_addr));
    int send_sock = socket(AF_INET,SOCK_DGRAM,0);
    if(send_sock < 0) //
    {
        printf("\n Socket creation error \n");
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = inet_addr(ip);
    // serv_addr.sin_addr.s_addr = INADDR_ANY;

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET,ip,&serv_addr.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    return send_sock;

}

int encode_drone_state(const prometheus_msgs::DroneState::ConstPtr& msg,int uav_id_)
{
    char *ptr = tcp_send_buf;

    // drone state length
    int ds_length = sizeof(int8_t)+3*3*sizeof(float);
    if(ds_length+1 > BUF_LEN)
    {
        ROS_ERROR("[communication_node] Topic is too large, please enlarge BUF_LEN");
        return -1;
    }

    // printf("uav%d encode: ",uav_id_);

    *((int8_t *)ptr) = uav_id_;
    // // printf(" %d",*ptr);
    ptr += sizeof(int8_t);
    
    //printf(" %f",msg->position[0]);
    *((float *)ptr) = msg->position[0];
    ptr += sizeof(float);
    // printf(" %f",msg->position[1]);
    *((float *)ptr) = msg->position[1];
    ptr += sizeof(float);
    // printf(" %f\n",msg->position[2]);
    *((float *)ptr) = msg->position[2];
    ptr += sizeof(float);

    // printf(" %f",msg->velocity[0]);
    *((float *)ptr) = msg->velocity[0];
    ptr += sizeof(float);
    // printf(" %f",msg->velocity[1]);
    *((float *)ptr) = msg->velocity[1];
    ptr += sizeof(float);
    // printf(" %f\n",msg->velocity[2]);
    *((float *)ptr) = msg->velocity[2];
    ptr += sizeof(float);

    // printf(" %f",msg->attitude[0]);
    *((float *)ptr) = msg->attitude[0];
    ptr += sizeof(float);
    // printf(" %f",msg->attitude[1]);
    *((float *)ptr) = msg->attitude[1];
    ptr += sizeof(float);
    // printf(" %f\n",msg->attitude[2]);
    *((float *)ptr) = msg->attitude[2];
    ptr += sizeof(float);
    
    return ptr - tcp_send_buf;
}

int decode_drone_state(prometheus_msgs::DroneState::Ptr &msg,int uav_id_)
{
    char *ptr = tcp_recv_buf;

    printf("uav%d decode: %d",uav_id_,*((int *)ptr));
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

    msg->velocity[0] = *((float *)ptr);
    printf(" %f",msg->velocity[0]);
    ptr += sizeof(float);
    msg->velocity[1] = *((float *)ptr);
    printf(" %f",msg->velocity[1]);
    ptr += sizeof(float);
    msg->velocity[2] = *((float *)ptr);
    printf(" %f\n",msg->velocity[2]);
    ptr += sizeof(float);

    msg->attitude[0] = *((float *)ptr);
    printf(" %f",msg->attitude[0]);
    ptr += sizeof(float);
    msg->attitude[1] = *((float *)ptr);
    printf(" %f",msg->attitude[1]);
    ptr += sizeof(float);
    msg->attitude[2] = *((float *)ptr);
    printf(" %f\n",msg->attitude[2]);
    ptr += sizeof(float);

    return ptr - tcp_recv_buf;
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

    // Forcefully attaching socket to the port
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
    // char str[INET_ADDRSTRLEN];
    // ROS_INFO( "Receive tcp connection from %s", inet_ntop(AF_INET, &address.sin_addr, str, sizeof(str)) );
}

void server_fun()
{
    int valread;
    // Connect
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
        valread = recv(recv_sock, tcp_recv_buf, BUF_LEN ,0);
        tcp_recv_buf[valread] = '\0';

        // cout<<tcp_recv_buf[0]<<endl;

        if(valread <= 0)
        {
            ROS_ERROR("Received message length <= 0, maybe connection has lost");
            close(recv_sock);
            close(server_fd);
            return;
        }

        cout<<tcp_recv_buf[0]<<"    "<<take_state<<endl;

        if((tcp_recv_buf[0] == '9') && (take_state == 0))
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "All the UAVs are disarmed and takeoff."<<endl;
            take_state = 1;
            // sleep(2);
            disarm_swarm();
            sleep(2);
            takeoff_swarm();
        }

        if((take_state == 1)&&(tcp_recv_buf[0] == '1')||(tcp_recv_buf[0] == '2')||(tcp_recv_buf[0] == '3')||(tcp_recv_buf[0] == '4'))
        {
            formation_num = tcp_recv_buf[0]-'0';
            pub_formation_command();
        }
        
        if((tcp_recv_buf[0] == '8') && (take_state == 1))
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "All the UAVs are armed and land."<<endl;
            // sleep(2);
            take_state = 0;
            land_swarm();
            sleep(8);
            arm_swarm();
            virtual_leader_pos << 0.0,0.0,1.0;
            virtual_leader_yaw = 0.0;
        }

        memset(tcp_recv_buf,0,sizeof(tcp_recv_buf));
    }

    close(recv_sock);
    close(server_fd);

}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg,int& uav_id_)
{
    int len = encode_drone_state(msg,uav_id_);
    sendto(send_sock,tcp_send_buf,len,0,(struct sockaddr*)&serv_addr,sizeof(serv_addr));
    // close(send_sock); 
}

int main(int argc, char **argv) 
{   

    ros::init(argc, argv, "ground_station_bridge");
    ros::NodeHandle nh("~");

    // swarm
    nh.param<int>("swarm_num", swarm_num, 1);
    nh.param<float>("formation_size", formation_size, 1.0);
    // broadcast
    nh.param<string>("ground_station_ip", tcp_ip, "192.168.1.21");
    nh.param<int>("tcp_port", TCP_PORT, 8887);
    // swarm control
    nh.param<int>("controller_num", controller_num, 0);

    virtual_leader_pos << 0.0,0.0,1.0;
    virtual_leader_yaw = 0.0;

    drone_state_msg.reset(new prometheus_msgs::DroneState);

    for(int i = 1; i <= swarm_num; i++) 
    {
        // uav id and name
        boost::format fmt1("uav%d");
        uav_name[i]= (fmt1%(i)).str();  
        uav_id[i]=i;
        // subscribe
        drone_state_sub[i] = nh.subscribe<prometheus_msgs::DroneState>("/"+uav_name[i] + "/prometheus/drone_state", 10, boost::bind(&drone_state_cb,_1,uav_id[i]));
        // publish
        command_pub[i] = nh.advertise<prometheus_msgs::SwarmCommand>("/"+uav_name[i] + "/prometheus/swarm_command", 10); //【发布】阵型    
    }

    // thread
    boost::thread recv_thd(server_fun);
    recv_thd.detach();
    ros::Duration(1).sleep(); // frequence

    // tcp connect: init socket

    send_sock = connect_to_ground_station(tcp_ip.c_str(), TCP_PORT);

    printf("start running\n");
    ros::spin();

    // close socket
    printf("closing sock\n");
    close(send_sock);


    // cout<<"hello"<<endl;

    return 0;
}
