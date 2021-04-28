#include "communication.h"

using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "ros_topic_2_tcp");
    ros::NodeHandle nh("~");

    return 0;
}