#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

#include <boost/thread/mutex.hpp>

#include <boost/thread/condition_variable.hpp>

#include <string>
#include <vector>

#include <map>

// Global Variables

extern int g_robot_id;
extern std::string g_tf_prefix;

extern std::string g_graph_name;

// 机器人位姿信息 
extern boost::mutex g_status_mutex;
extern double g_x_robot;
extern double g_y_robot;
extern double g_th_robot;

// 地图中各节点空闲率信息 
extern boost::mutex g_idleness_mutex;
extern double * g_last_visit;

// 其他机器人的目标点信息 
// 其他机器人要到达下一个目标点的信息 
extern boost::mutex g_intention_mutex;
extern std::map<int, int> g_intention_mate;
extern std::map<int, double> g_intention_decision_time;
extern std::map<int, double> g_intention_estimate_time;
// 其他机器人规划要到达的目标点信息 
extern boost::mutex g_goal_mutex;
extern std::map<int, int> g_goal_mate;
extern std::map<int, double> g_goal_decision_time;
extern std::map<int, double> g_goal_estimate_time;


// 用于控制机器人巡逻任务开始和停止的条件变量和互斥量 
extern bool g_is_start;
extern boost::mutex g_cond_start_mutex;  // 用于条件变量的互斥量 
extern boost::condition_variable_any g_cond_start;  // 条件变量 

extern bool g_is_stop;
extern boost::mutex g_cond_stop_mutex;  // 用于条件变量的互斥量 
extern boost::condition_variable_any g_cond_stop;  // 条件变量 

// 存储机器人的移动速度估计值，用于实现异构多机器人的巡逻(速度不同) 
extern double g_vel_robot;

// 存储机器人巡逻任务的多步规划，即依次需要到达的节点 
extern int g_plan_vertex_num;
extern std::vector<int> g_plan_vertex_list;


#endif  // GLOBAL_VARIABLES_H_
