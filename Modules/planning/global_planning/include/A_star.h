// 路径规划算法A*
// 根据得到的地图规划一条路径
#ifndef _ASTAR_H
#define _ASTAR_H

#include <ros/ros.h>
#include <Eigen/Eigen>	// 内置矩阵库
#include <iostream>	
#include <queue>    // STL队列
#include <string>   // 字符串库
#include <unordered_map>	// STL容器
#include <sstream>	// 字符串流，常用于格式转化

#include <sensor_msgs/PointCloud2.h>    // 二维点云数据
#include <nav_msgs/Path.h>  // 路径数据

#include "occupy_map.h" // 包含了地图
#include "tools.h"
#include "message_utils.h"

#define NODE_NAME "Global_Planner [Astar]"  // 定义节点名称

namespace Global_Planning
{
// 为什么不用enum？
#define IN_CLOSE_SET 'a'    // 在close_set中
#define IN_OPEN_SET 'b'     // 在open_set中
#define NOT_EXPAND 'c'      // 既不在close_set也不在open_set中
#define inf 1 >> 30         // 定义无穷大，1右移30位 （但是我看人家都是左移？）

extern ros::Publisher message_pub;

class Node  // 位置节点，用于记录某个位置的信息
{
    public:
        /* -------------------- */
        Eigen::Vector3i index;  // 三维索引号
        Eigen::Vector3d position;   // 三维坐标
        double g_score, f_score;    // g_score表示从初始点到该点的总代价，f_score表示从初始点经过该点到达目标点的估计代价
        Node* parent;   // 节点的父节点，用于记录路径
        char node_state;    // in_close_set/in_open_set/not_expand

        double time;  // dyn
        int time_idx;

        Node()  // 构造函数
        {
          parent = NULL;    // 初始化父节点为0
          node_state = NOT_EXPAND;  // 初始化节点状态为NOT_EXPAND，即未被访问
        }
        ~Node(){};  // 析构函数不做任何处理
};
typedef Node* NodePtr;  // 重命名节点指针

// 为什么这么麻烦，不能直接比较吗
// 因为优先队列中需要该函数进行初始化，只能这样做
class NodeComparator0   // 伪函数，本质上是一个类，只是重载了()运算符
{
    public:
        // 如果node1的总体估计代价比node2的总体估计代价大，就返回true
        // 这会使得优先队列中的节点是按f_score从小到大排列的
        bool operator()(NodePtr node1, NodePtr node2)
        {
          return node1->f_score > node2->f_score;
        }
};

template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t>    // 什么东西？ 节点初始化的函数？
{
    std::size_t operator()(T const& matrix) const
    {
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i)
        {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class NodeHashTable0    // 三维索引到节点指针的映射
{
  private:
      /* data */
      std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;

  public:
      NodeHashTable0(/* args */)
      {
      }
      ~NodeHashTable0()
      {
      }
      void insert(Eigen::Vector3i idx, NodePtr node)    // 插入一个索引
      {
        data_3d_.insert(std::make_pair(idx, node));
      }

      NodePtr find(Eigen::Vector3i idx)     // 根据索引查找节点
      {
        auto iter = data_3d_.find(idx);
        return iter == data_3d_.end() ? NULL : iter->second;
      }

      void clear()  // 清除所有键值对
      {
        data_3d_.clear();
      }
};


class Astar
{
    private:
        // 备选路径点指针容器
        std::vector<NodePtr> path_node_pool_;
        // 使用节点计数器、迭代次数计数器
        int use_node_num_, iter_num_;
        // 扩展的节点
        NodeHashTable0 expanded_nodes_;
        // open set （根据规则已排序好）
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
        // 最终路径点容器
        std::vector<NodePtr> path_nodes_;  

        // 参数
        // 启发式参数
        double lambda_heu_;
        // 最大搜索次数
        int max_search_num;
        // tie breaker
        double tie_breaker_;
        // 是否是2D搜索
        int is_2D;
        double fly_height;

        /* ---------- record data ---------- */
        // 目标点
        Eigen::Vector3d goal_pos;

        // 地图相关
        std::vector<int> occupancy_buffer_;  
        double resolution_, inv_resolution_;
        Eigen::Vector3d origin_, map_size_3d_;
        bool has_global_point;

        // 辅助函数
        Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
        void indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos);  // 节点索引转换为坐标
        void retrievePath(NodePtr end_node);    // 搜索到一个路径后，使用该函数来获取

        // 搜索启发函数，三种形式，选用其中一种即可
        double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

    public:
        Astar(){}   // 构造函数，不做任何操作
        ~Astar();   // 析构函数

        enum
        {
          REACH_END = 1,    // 能到达
          NO_PATH = 2   // 不能到达
        };

        // 占据图类
        Occupy_map::Ptr Occupy_map_ptr;

        // 重置
        void reset();
        // 初始化
        void init(ros::NodeHandle& nh);
        // 检查安全性
        bool check_safety(Eigen::Vector3d &cur_pos, double safe_distance);
        // 搜索
        int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
        // 返回路径
        std::vector<Eigen::Vector3d> getPath();
        // 返回ros消息格式的路径
        nav_msgs::Path get_ros_path();
        // 返回访问过的节点
        std::vector<NodePtr> getVisitedNodes();

        typedef std::shared_ptr<Astar> Ptr;

};


}

#endif
