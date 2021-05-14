/*  
    动态的坐标系相对姿态发布(一个坐标系相对于另一个坐标系的相对姿态是不断变动的)
    需求: 无人机的geometry_msgs/PoseStamped为位置坐标，有一个世界坐标系，将两个根据无人机位置坐标
    发布tf坐标相对关系，可以在rivz中查看
    实现分析:
        1.无人机本身不但可以看作坐标系，也是世界坐标系中的一个坐标点
        2.订阅 geometry_msgs/PoseStamped ,可以获取无人机在世界坐标系的x,y,x和四元数
        3.将 pose 信息转换成坐标系相对信息并发布
    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建订阅对象
        5.回调函数处理订阅到的数据(实现TF广播)
            5-1.创建 TF 广播器
            5-2.创建 广播的数据(通过 pose 设置)
            5-3.广播器发布数据
        6.spin
*/
// 1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"  //发布动态坐标关系
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
using namespace std;
ros::NodeHandle nh("~");

//位资信息转换成坐标系相对关系并发布
void doPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //  5-1.创建 TF 广播器   加入static使每次回调使用同一个pub，否则将一直重复创建
    static tf2_ros::TransformBroadcaster broadcaster;
    //  5-2.创建 广播的数据(通过 pose 设置)  被发布的数据
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";  //相对于世界坐标系
    tfs.header.stamp = ros::Time::now();  //时间戳
    
    string uav_name;
    nh.param<string>("uav_name", uav_name, "/none");  
    //  |----坐标系 ID
    tfs.child_frame_id = uav_name;  //子坐标系，无人机的坐标系

    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = msg->pose.position.x;
    tfs.transform.translation.y = msg->pose.position.y;
    tfs.transform.translation.z = msg->pose.position.z; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置  
    tf2::Quaternion qtn;
    // qtn.setRPY(0,0,pose->theta);  //无人机的欧拉角  俯仰角、翻滚角、theta为z轴偏航角
    tfs.transform.rotation.x = msg->pose.orientation.x;
    tfs.transform.rotation.y = msg->pose.orientation.y;
    tfs.transform.rotation.z = msg->pose.orientation.z;
    tfs.transform.rotation.w = msg->pose.orientation.w;


    //  5-3.广播器发布数据
    broadcaster.sendTransform(tfs);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,""); //设置编码
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"pub_tf");
    // 3.创建 ROS 句柄
    // 4.创建订阅对象:订阅无人机的世界坐标
    string uav_name;
    bool sim_mode;
    nh.param<string>("uav_name", uav_name, "/none");
    nh.param<bool>("sim_mode", sim_mode, true);

    if(sim_mode)
    {
        ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ uav_name + "/pose",1000,doPose);
    }else
    {
        ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ uav_name + "/pose",1000,doPose);
    }
    
    ros::spin();
    return 0;
}