#ifndef TASK_EXECUTION_H_
#define TASK_EXECUTION_H_

#include <ros/ros.h>

#include "getgraph.h"
#include "algorithms.h"

#include "my_algorithms.h"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <string>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// edge definition 
typedef struct {
    unsigned int id_1, id_2;
    double x_1, y_1;
    double x_2, y_2;
    double cost;
} edge;


// 任务执行线程函数  函数声明 
void execution_thread();

class TaskExecution
{
public:
    TaskExecution()
    {
        robot_id_ = g_robot_id;
        
        // Create the string "robot_X/move_base" 
        move_base_str_ = g_tf_prefix + "/move_base";
        
        std::cout << move_base_str_ << std::endl;
        
        ac = new MoveBaseClient(move_base_str_, true);
        
        ac->waitForServer(ros::Duration(5));
        
        // 读取 .graph 文件，构造 vertex_web 
        std::string graph_name = g_graph_name;
        
        // Check Graph Dimension 
        dimension = GetGraphDimension( graph_name.c_str() );
        // Create Structure to save the Graph Info 
        vertex_web = new vertex[dimension];
        // Get the Graph info from the Graph File 
        GetGraphInfo( vertex_web, dimension, graph_name.c_str() );
        
        vel_robot = g_vel_robot;  // 对机器人移动速度的估计 
        
        // 提取图中所有的边 edge，要避免重复 
        edge_list.clear();
        for(int i=0; i<dimension; ++i)
        {
            for(int j=0; j<vertex_web[i].num_neigh; ++j)
            {
                edge one_edge;
                one_edge.id_1 = vertex_web[i].id;
                one_edge.id_2 = vertex_web[i].id_neigh[j];
                one_edge.cost = vertex_web[i].cost[j];
                bool is_repeated = false;
                int num_edge = edge_list.size();
                for(int k=0; k<num_edge; ++k)
                {
                    if( (one_edge.id_1==edge_list[k].id_1) && (one_edge.id_2==edge_list[k].id_2) )  is_repeated = true;
                    if( (one_edge.id_1==edge_list[k].id_2) && (one_edge.id_2==edge_list[k].id_1) )  is_repeated = true;
                }
                if(is_repeated==false)
                {
                    one_edge.x_1 = vertex_web[i].x;
                    one_edge.y_1 = vertex_web[i].y;
                    one_edge.x_2 = vertex_web[vertex_web[i].id_neigh[j]].x;
                    one_edge.y_2 = vertex_web[vertex_web[i].id_neigh[j]].y;
                    edge_list.push_back(one_edge);
                }
            }
        }
        edge_num = edge_list.size();
        edge_avg = 0;
        edge_min = edge_list[0].cost;
        edge_max = edge_list[0].cost;
        for(int k=0; k<edge_num; ++k)
        {
            edge_avg += edge_list[k].cost;
            if(edge_min > edge_list[k].cost)  edge_min = edge_list[k].cost;
            if(edge_max < edge_list[k].cost)  edge_max = edge_list[k].cost;
        }
        edge_avg = edge_avg / edge_num;
//        std::cout << "edge_avg: " << edge_avg << std::endl;
        
        // plan_vertex_num 等变量的初始化在函数 void TaskExecution::execute_task() 中进行 
        
        // 用于 heuristic_pathfinder_conscientious_cognitive 算法 
        path = new unsigned int [dimension];
        elem_s_path = 0;
        i_path = 0;
        
        // 初始化空闲率信息 idleness, last_visit 
        last_visit = new double[dimension];
        instantaneous_idleness = new double[dimension];
        estimate_last_visit = new double[dimension];
        for(int i=0; i<dimension; ++i)
        {
            last_visit[i] = 0.0;
            instantaneous_idleness[i] = 0.0;
            estimate_last_visit[i] = 0.0;
        }
        
        goal_reached_wait = 2.0;  // 到达巡逻节点后的等待时间，表示在此处执行检测操作所需的时间 
        
        // check whether the robot is stuck or not every 20 seconds 
        check_time = 0;
        check_x_robot = 0;
        check_y_robot = 0;
    }
    
    ~TaskExecution()
    {
        delete ac;
        delete [] vertex_web;
        delete [] last_visit;
        delete [] instantaneous_idleness;
        delete [] estimate_last_visit;
        delete [] path;
    }

    void execute_task();
    
    void send_goal(unsigned int next_vertex, double x_robot, double y_robot);
    
    unsigned int identify_vertex(double x_robot, double y_robot);
    
    unsigned int decide_next_vertex(double x_robot, double y_robot);
    
    void publish_idleness(int vertex_id, double current_time);
    void publish_intention(int vertex_id, double current_time, double x_robot, double y_robot);
    
    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void goalActiveCallback();
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

private:
    int robot_id_;
    
    std::string move_base_str_;
    
    MoveBaseClient *ac;  // action client for reaching target goals 
    
    unsigned int dimension;  // Graph Dimension 
    vertex *vertex_web;
    
    unsigned int current_vertex;
    unsigned int next_vertex;
    
    double * last_visit;
    double * instantaneous_idleness;
    
    double * estimate_last_visit;
    
    double goal_reached_wait;
    
    double vel_robot;
    
    std::vector<edge> edge_list;
    int edge_num;
    double edge_max;
    double edge_min;
    double edge_avg;
    
    int plan_vertex_num;
    std::vector<int> plan_vertex_list;
    int plan_vertex_i;  // 机器人当前所在的节点编号 
    
    // 用于 heuristic_pathfinder_conscientious_cognitive 算法 
    unsigned int * path;
    unsigned int elem_s_path;
    unsigned int i_path;
    
    // check whether the robot is stuck or not every 20 seconds 
    double check_time;
    double check_x_robot;
    double check_y_robot;
    
};


#endif  // TASK_EXECUTION_H_
