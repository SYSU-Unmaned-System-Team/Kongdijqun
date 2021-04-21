#include <occupy_map.h>

namespace Global_Planning
{
// 初始化函数
void Occupy_map::init(ros::NodeHandle& nh)
{
    // 集群数量
    nh.param<int>("swarm_num", swarm_num, 1);
    // 无人机编号 1号无人机则为1
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<string>("uav_name", uav_name, "/uav0");
    // 全局地图点云指针
    global_point_cloud_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // 传入点云指针（临时指针）
    input_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // tf变换后点云指针（临时指针）
    transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // 过滤后点云指针（临时指针）
    pcl_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // 局部地图滑窗指示器
    st_it = 0;
    // 存储的上一帧odom
    f_x = f_y = f_z = f_pitch = f_yaw = f_roll = 0.0;
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    is_2D = true; 
    // 2D规划时,定高高度
    nh.param("global_planner/fly_height_2D", fly_height_2D, 1.0);
    // 地图原点
    nh.param("map/origin_x", origin_(0), -5.0);
    nh.param("map/origin_y", origin_(1), -5.0);
    nh.param("map/origin_z", origin_(2), 0.0);
    // 地图实际尺寸，单位：米
    nh.param("map/map_size_x", map_size_3d_(0), 10.0);
    nh.param("map/map_size_y", map_size_3d_(1), 10.0);
    nh.param("map/map_size_z", map_size_3d_(2), 5.0);
    // localmap slide window
    nh.param("map/queue_size", queue_size, 20);
    // show border
    nh.param("map/border", show_border, false);
    // 地图分辨率，单位：米
    nh.param("map/resolution", resolution_,  0.2);
    // 地图膨胀距离，单位：米
    nh.param("map/inflate", inflate_,  0.3);

    // 发布 地图rviz显示
    global_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(uav_name + "/prometheus/planning/global_pcl",  10); 
    // 发布膨胀后的点云
    inflate_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(uav_name + "/prometheus/planning/global_inflate_pcl", 1);
 
    // 发布二维占据图？
    // 发布膨胀后的二维占据图？

    this->inv_resolution_ = 1.0 / resolution_;
    for (int i = 0; i < 3; ++i)
    {
        // 占据图尺寸 = 地图尺寸 / 分辨率
        grid_size_(i) = ceil(map_size_3d_(i) / resolution_);
    }
    
    // 占据容器的大小 = 占据图尺寸 x*y*z
    occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);

    min_range_ = origin_;
    max_range_ = origin_ + map_size_3d_;   

    // 对于二维情况，重新限制点云高度
    if(is_2D)
    {
        min_range_(2) = fly_height_2D - resolution_;
        max_range_(2) = fly_height_2D + resolution_;
    }

    // 生成地图边界：点云形式
    border.width = 4000;
    border.height = 1;
    border.points.resize(4000);
    for(int i=0 ; i<1000; i++) //todo: auto border
    {
        border.points[i].x = min_range_(0)+i*(max_range_(0)-min_range_(0))/1000.0;
        border.points[i].y = min_range_(1);
        border.points[i].z = min_range_(2);

        border.points[i+1000].x = min_range_(0)+i*(max_range_(0)-min_range_(0))/1000.0;
        border.points[i+1000].y = max_range_(1);
        border.points[i+1000].z = min_range_(2);

        border.points[i+2000].x = min_range_(0);
        border.points[i+2000].y = min_range_(1)+i*(max_range_(1)-min_range_(1))/1000.0;
        border.points[i+2000].z = min_range_(2);

        border.points[i+3000].x = max_range_(0);
        border.points[i+3000].y = min_range_(1)+i*(max_range_(1)-min_range_(1))/1000.0;
        border.points[i+3000].z = min_range_(2);
    }
}

// 地图更新函数 - 输入：全局点云
void Occupy_map::map_update_gpcl(const sensor_msgs::PointCloud2ConstPtr & global_point)
{
    // rec global map
    pcl::fromROSMsg(*global_point,*input_point_cloud);
    global_point_cloud_map = input_point_cloud;
    // map border
    if(show_border)
    {
        *transformed_cloud = *global_point_cloud_map + border;
    }
    has_global_point = true;
}

// 工具函数：合并局部地图 - 输入：odom以及局部点云
void Occupy_map::local_map_merge_odom(const nav_msgs::Odometry & odom)
{
    // 从odom中取得6DOF
    double x, y, z, roll, pitch, yaw;
    // 平移（xyz）
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;
    // 旋转（从四元数到欧拉角）
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);    
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // 即使不移动,时间达到阈值,也会更新地图
    static int update_num = 0;
    update_num++;
    if (update_num > 40)
    {
        update_num = 0;
    }

    // 只有移动了一定距离，才接收局部点云（过滤过多的点云）；达成指定高度才建图
    bool pos_change = (abs(f_x-x)>0.1 || abs(f_y-y)>0.1) && (fly_height_2D-0.1<z && z<fly_height_2D+0.1);
    // 只有在无人机平稳（角度足够小）时更新点云 （此条件必须满足）
    // 0.09 约等于5度
    // 将无人机飞行时 角度限制在5度以内
    // 偏航角???
    bool ang_change = abs(f_pitch-pitch)<0.08 && abs(f_roll-roll)<0.08 && abs(f_yaw-yaw)<0.08;
    // 合并局部点云，形成局部地图, todo: incremental merge?
    if((global_point_cloud_map == nullptr || pos_change || update_num == 40) && ang_change) 
    {
        // 为滑窗的点云累计odom
        map<int,pcl::PointCloud<pcl::PointXYZ>>::iterator iter;
        for(iter = point_cloud_pair.begin(); iter != point_cloud_pair.end(); iter++)
        {
            pcl::transformPointCloud(iter->second,*transformed_cloud,pcl::getTransformation(f_x-x, f_y-y, f_z-z, f_roll-roll, f_pitch-pitch, f_yaw-yaw));
            iter->second = *transformed_cloud;
        }

        // 局部地图滑窗, 大小为: $queue_size
        point_cloud_pair[st_it] = *input_point_cloud; // 加入新点云到滑窗
        st_it = (st_it + 1) % queue_size; // 指向下一个移除的点云位置

        // 累计局部地图
        pcl_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
        for(iter = point_cloud_pair.begin(); iter != point_cloud_pair.end(); iter++)
        {
            *pcl_ptr += iter->second;
        }

        // remove outlier
        sor.setInputCloud(pcl_ptr);
        sor.setMeanK(20); // kNN的最少的邻居数
        sor.setStddevMulThresh(1.0); // threshold
        sor.setNegative(false);
        sor.filter(*global_point_cloud_map);

        // downsample
        vg.setInputCloud(global_point_cloud_map);
        vg.setLeafSize(0.05f, 0.05f, 0.5f); // 下采样叶子节点大小（3D容器）
        vg.filter(*pcl_ptr);

        // border
        if(show_border)
        {
            // tf to lidar frame
            pcl::transformPointCloud(border,*transformed_cloud,pcl::getTransformation(f_x-x, f_y-y, f_z-z, 0, 0, 0));
            border = *transformed_cloud;
            // tf global map
            *transformed_cloud = *pcl_ptr + border;
        }

        // astar global map in world frame
        pcl::transformPointCloud(*pcl_ptr,*global_point_cloud_map,pcl::getTransformation(x, y, z, 0, 0, 0));

        // store odom data
        f_x = x;
        f_y = y;
        f_z = z;
        f_roll = roll;
        f_pitch = pitch;
        f_yaw = yaw;
        // global map flag
        has_global_point = true;
    }
    else
    {
        has_global_point = false;
        //cout << "map update failed."<< endl;
    } 
}

// 地图更新函数 - 输入：局部点云
void Occupy_map::map_update_lpcl(const sensor_msgs::PointCloud2ConstPtr & local_point, const nav_msgs::Odometry & odom)
{
    // 由sensor_msgs::PointCloud2 转为 pcl::PointCloud<pcl::PointXYZ>
    pcl::fromROSMsg(*local_point,*input_point_cloud);
    // 将局部点云融合至全局点云
    local_map_merge_odom(odom);
}

// 地图更新函数 - 输入：laser
void Occupy_map::map_update_laser(const sensor_msgs::LaserScanConstPtr & local_point, const nav_msgs::Odometry & odom)
{
    // 参考网页:http://wiki.ros.org/laser_geometry
    // sensor_msgs::LaserScan 转为 sensor_msgs::PointCloud2 格式
    projector_.projectLaser(*local_point, input_laser_scan);
    // 再由sensor_msgs::PointCloud2 转为 pcl::PointCloud<pcl::PointXYZ>
    pcl::fromROSMsg(input_laser_scan,*input_point_cloud);
    // 将局部点云融合至全局点云
    local_map_merge_odom(odom);
}

// 当global_planning节点接收到点云消息更新时，进行设置点云指针并膨胀
// Astar规划路径时，采用的是此处膨胀后的点云（setOccupancy只在本函数中使用）
void Occupy_map::inflate_point_cloud(void)
{
    if(!has_global_point)
    {
        // pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Occupy_map [inflate point cloud]: don't have global point, can't inflate!\n");
        return;
    }

    // 发布未膨胀点云
    sensor_msgs::PointCloud2 global_env_;
    if(show_border)
    {
        pcl::toROSMsg(*transformed_cloud,global_env_);
        if(queue_size==-1)
        {
            global_env_.header.frame_id = "world";
        }
        else
        {
            global_env_.header.frame_id = uav_name+"/lidar_link";
        }
    }
    else
    {
        pcl::toROSMsg(*global_point_cloud_map,global_env_);
        global_env_.header.frame_id = "world";
    }
    global_pcl_pub.publish(global_env_);

    //记录开始时间
    ros::Time time_start = ros::Time::now();

    // 转化为PCL的格式进行处理
    pcl::PointCloud<pcl::PointXYZ> latest_global_cloud_ = *global_point_cloud_map;

    //printf("time 1 take %f [s].\n",   (ros::Time::now()-time_start).toSec());

    if ((int)latest_global_cloud_.points.size() == 0)  
    {return;}

    pcl::PointCloud<pcl::PointXYZ> cloud_inflate_vis_;
    cloud_inflate_vis_.clear();

    // 膨胀格子数 = 膨胀距离/分辨率
    // ceil返回大于或者等于指定表达式的最小整数
    const int ifn = ceil(inflate_ * inv_resolution_);

    pcl::PointXYZ pt_inf;
    Eigen::Vector3d p3d, p3d_inf;

    // 遍历全局点云中的所有点
    for (size_t i = 0; i < latest_global_cloud_.points.size(); ++i) 
    {
        // 取出第i个点
        p3d(0) = latest_global_cloud_.points[i].x;
        p3d(1) = latest_global_cloud_.points[i].y;
        p3d(2) = latest_global_cloud_.points[i].z;
        
        // 若取出的点不在地图内（膨胀时只考虑地图范围内的点）
        if(!isInMap(p3d))
        {
            continue;
        }
        
        // 根据膨胀距离，膨胀该点
        for (int x = -ifn; x <= ifn; ++x)
            for (int y = -ifn; y <= ifn; ++y)
                for (int z = -ifn; z <= ifn; ++z) 
                {
                    // 为什么Z轴膨胀一半呢？ z 轴其实可以不膨胀
                    p3d_inf(0) = pt_inf.x = p3d(0) + x * resolution_;
                    p3d_inf(1) = pt_inf.y = p3d(1) + y * resolution_;
                    p3d_inf(2) = pt_inf.z = p3d(2) + 0.5 * z * resolution_;

                    // 若膨胀的点不在地图内（膨胀时只考虑地图范围内的点）
                    if(!isInMap(p3d_inf))
                    {
                        continue;
                    }

                    cloud_inflate_vis_.push_back(pt_inf);

                    // 设置膨胀后的点被占据
                    this->setOccupancy(p3d_inf, 1);
                }
    }

    // 转化为ros msg发布
    sensor_msgs::PointCloud2 map_inflate_vis;
    pcl::toROSMsg(cloud_inflate_vis_, map_inflate_vis);
    map_inflate_vis.header.frame_id = "world";

    inflate_pcl_pub.publish(map_inflate_vis);

    static int exec_num=0;
    exec_num++;

    // 此处改为根据循环时间计算的数值
    if(exec_num == 50)
    {
        // 膨胀地图效率与地图大小有关
        char message_chars[256];
        sprintf(message_chars, "inflate global point take %f [s].", (ros::Time::now()-time_start).toSec());
        message = message_chars;
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);

        exec_num=0;
    }  
}

void Occupy_map::setOccupancy(Eigen::Vector3d &pos, int occ) 
{
    if (occ != 1 && occ != 0) 
    {
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "occ value error!\n");
        return;
    }

    if (!isInMap(pos))
    {
        return;
    }

    Eigen::Vector3i id;
    posToIndex(pos, id);

    // 设置为占据/不占据 索引是如何索引的？ [三维地图 变 二维数组]
    // 假设10*10*10米，分辨率1米，buffer大小为 1000 （即每一个占格都对应一个buffer索引）
    // [0.1,0.1,0.1] 对应索引为[0,0,0]， buffer索引为 0  
    // [9.9,9.9,9.9] 对应索引为[9,9,9]， buffer索引为 900+90+9 = 999
    occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)] = occ;
}

bool Occupy_map::isInMap(Eigen::Vector3d pos) 
{
    // min_range就是原点，max_range就是原点+地图尺寸
    // 比如设置0,0,0为原点，[0,0,0]点会被判断为不在地图里
    //　同时　对于２Ｄ情况，超出飞行高度的数据也会认为不在地图内部
    if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 || pos(2) < min_range_(2) + 1e-4) 
    {
        return false;
    }

    if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 || pos(2) > max_range_(2) - 1e-4) 
    {
        return false;
    }

    return true;
}

bool Occupy_map::check_safety(Eigen::Vector3d& pos, double check_distance)
{
    if(!isInMap(pos))
    {
        // 当前位置点不在地图内
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "[check_safety]: the odom point is not in map\n");
        return 0;
    }
    Eigen::Vector3i id;
    posToIndex(pos, id);
    Eigen::Vector3i id_occ;
    Eigen::Vector3d pos_occ;

    int check_dist_xy = int(check_distance/resolution_);
    int check_dist_z=0;
    int cnt=0;
    for(int ix=-check_dist_xy; ix<=check_dist_xy; ix++){
        for(int iy=-check_dist_xy; iy<=check_dist_xy; iy++){
            for(int iz=-check_dist_z; iz<=check_dist_z; iz++){
                id_occ(0) = id(0)+ix;
                id_occ(1) = id(1)+iy;
                id_occ(2) = id(2)+iz;
                indexToPos(id_occ, pos_occ);
                if(!isInMap(pos_occ)){
                    // printf("[check_safety]: current odom is near the boundary of the map\n");
                    // pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "[check_safety]: current odom is near the boundary of the map\n");
                    return 0;
                }
                if(getOccupancy(id_occ)){
                    // printf("[check_safety]: current state is dagerous, the pos [%d, %d, %d], is occupied\n", ix, iy, iz);
                    cnt++;             
                }
            }
        }
    }
    if(cnt>5){
        return 0;
    }
    return 1;

}

void Occupy_map::posToIndex(Eigen::Vector3d &pos, Eigen::Vector3i &id) 
{
    for (int i = 0; i < 3; ++i)
    {
        id(i) = floor((pos(i) - origin_(i)) * inv_resolution_);
    }
       
}

void Occupy_map::indexToPos(Eigen::Vector3i &id, Eigen::Vector3d &pos) 
{
    for (int i = 0; i < 3; ++i)
    {
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
    }
}

int Occupy_map::getOccupancy(Eigen::Vector3d &pos) 
{
    if (!isInMap(pos))
    {
        return -1;
    }
        
    Eigen::Vector3i id;
    posToIndex(pos, id);

    return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}

int Occupy_map::getOccupancy(Eigen::Vector3i &id) 
{
    if (id(0) < 0 || id(0) >= grid_size_(0) || id(1) < 0 || id(1) >= grid_size_(1) || id(2) < 0 ||
        id(2) >= grid_size_(2))
    {
        return -1;
    }
        
    return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}
}
