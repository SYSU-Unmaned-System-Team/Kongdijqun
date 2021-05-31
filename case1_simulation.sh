#!/bin/bash
# source /home/dji/Kongdijiqun/devel/setup.bash

# 启动gazebo,rviz
roslaunch prometheus_gazebo sitl_cxy_case1_40uav_0.launch & sleep 10s;

# 启动1-10架无人机
roslaunch prometheus_gazebo sitl_cxy_case1_40uav_1.launch & sleep 20s;

# 启动11-20架无人机
roslaunch prometheus_gazebo sitl_cxy_case1_40uav_2.launch & sleep 20s;

# 启动21-30架无人机
roslaunch prometheus_gazebo sitl_cxy_case1_40uav_3.launch & sleep 20s;

# 启动31-40架无人机
roslaunch prometheus_gazebo sitl_cxy_case1_40uav_4.launch & sleep 20s;

# 启动阵型切换节点、地面站节点
roslaunch prometheus_gazebo sitl_cxy_case1_40uav_5.launch

# rosbag record -a


