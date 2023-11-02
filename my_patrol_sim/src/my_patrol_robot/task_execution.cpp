#include "global_variables.h"

#include "task_execution.h"

#include <lcm/lcm-cpp.hpp>
#include "exlcm/command_t.hpp"
#include "exlcm/idleness_t.hpp"
#include "exlcm/intention_t.hpp"
#include "exlcm/goal_t.hpp"
#include "exlcm/plan_t.hpp"

#include <cmath>


// 任务执行线程函数 
void execution_thread()
{
    TaskExecution my_task;
    while (true)
    {
        // 等待开始信号 
        g_is_start = false;
        {
            boost::mutex::scoped_lock cond_start_lock(g_cond_start_mutex);
            while ( g_is_start == false )
            {
                g_cond_start.wait(g_cond_start_mutex);
            }
        }
        g_is_stop = false;
        // 开始执行巡逻任务 ... 
        my_task.execute_task();
    }
}


// 巡逻任务执行函数 
void TaskExecution::execute_task()
{
    printf("%s received START COMMAND \n", move_base_str_.c_str());
    
    // 决策发布第一个任务 
    
    // 初始化地图中各节点的空闲率信息 
    {
        double current_time = ros::Time::now().toSec();
        boost::mutex::scoped_lock idleness_lock(g_idleness_mutex);
        for(int i=0; i<dimension; ++i)
        {
            g_last_visit[i] = current_time;
            last_visit[i] = current_time;
            instantaneous_idleness[i] = 0.0;
        }
    }
    
    // 机器人当前的位姿 
    double x_robot, y_robot, th_robot;
    {
        boost::mutex::scoped_lock status_lock(g_status_mutex);
        x_robot = g_x_robot;
        y_robot = g_y_robot;
        th_robot = g_th_robot;
    }
    
    // 确定机器人当前所在的节点（距离机器人当前位置最近的节点） 
    current_vertex = identify_vertex(x_robot, y_robot);
    
    next_vertex = current_vertex;
    
    
    // 如果存在分区巡逻计划，则复制计划，并将机器人的初始目标点设置为对应分区的起点 
    if ( g_plan_vertex_num == 0 )
    {
        plan_vertex_num = 0;
        plan_vertex_i = 0;
    }
    else if ( g_plan_vertex_num > 0 )
    {
        plan_vertex_num = g_plan_vertex_num;
        plan_vertex_i = 0;
        plan_vertex_list.clear();
        for (int i=0; i<plan_vertex_num; ++i)
        {
            plan_vertex_list.push_back( g_plan_vertex_list[i] );
        }
//        std::cout << "plan_vertex_num = " << plan_vertex_num << std::endl;
        next_vertex = plan_vertex_list[0];
    }
    else
    {
        std::cout << "Caution: plan_vertex_num error !" << std::endl;
    }
    
    
    // 发布 intention 更新消息 
    publish_intention(next_vertex, ros::Time::now().toSec(), x_robot, y_robot);
    
    send_goal(next_vertex, x_robot, y_robot);
    
    // 等待停止信号，在收到停止信号之前，会通过 send_goal 和 goalDoneCallback 函数 持续执行巡逻任务 
    {
        boost::mutex::scoped_lock cond_stop_lock(g_cond_stop_mutex);
        while ( g_is_stop == false )
        {
            g_cond_stop.wait(g_cond_stop_mutex);
        }
    }
    // 取消正在执行的任务 
    ac->cancelAllGoals();
    // Wait for the action to return
    ac->waitForResult(ros::Duration(5));
}


// 任务完成时，调用 done 函数 (无论任务执行成功或是失败，即使任务被终止，也会调用)，可用于执行任务结束后的收尾工作 
void TaskExecution::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    // 如果已收到停止信号，则函数直接返回 
    if ( g_is_stop == true )
    {
        printf("%s received STOP COMMAND \n", move_base_str_.c_str());
        return;
    }
    
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("%s has reached the goal ( the vertex %u )", move_base_str_.c_str(), next_vertex);
//        ROS_INFO("%s: Goal reached ... WAITING %.2f sec", move_base_str_.c_str(), goal_reached_wait);
        ros::Duration delay(goal_reached_wait);  // wait after goal is reached 
        delay.sleep();
//        ROS_INFO("%s: Goal reached ... WAITING DONE", move_base_str_.c_str());
        
        double current_time = ros::Time::now().toSec();
        
        // 发布 idleness 更新消息 
        publish_idleness(next_vertex, current_time);
        
        // 更新地图中各节点的空闲率信息 
        {
            boost::mutex::scoped_lock idleness_lock(g_idleness_mutex);
            g_last_visit[next_vertex] = current_time;
            for(int i=0; i<dimension; ++i)
            {
                last_visit[i] = g_last_visit[i];
                instantaneous_idleness[i] = current_time - last_visit[i];
            }
        }
        
        current_vertex = next_vertex;
        
        double x_robot = ( vertex_web[current_vertex].x ) * RESOLUTION;
//        double y_robot = ( HEIGHT_PX - vertex_web[current_vertex].y ) * RESOLUTION;
        double y_robot = ( vertex_web[current_vertex].y ) * RESOLUTION;
        
        // 决策发布下一个任务 
        next_vertex = decide_next_vertex(x_robot, y_robot);
        
        // 发布 intention 更新消息 
        publish_intention(next_vertex, ros::Time::now().toSec(), x_robot, y_robot);
        
//        send_goal(next_vertex, current_vertex);
        send_goal(next_vertex, x_robot, y_robot);
    }
    else
    {
        ROS_INFO("%s failed for some reason", move_base_str_.c_str());
        
        // 机器人当前的位姿 
        double x_robot, y_robot, th_robot;
        {
            boost::mutex::scoped_lock status_lock(g_status_mutex);
            x_robot = g_x_robot;
            y_robot = g_y_robot;
            th_robot = g_th_robot;
        }
        
        // random move in order to avoid stuck 
        // 取消正在执行的任务 
//        ac->cancelAllGoals();
        // Wait for the action to return 
//        ac->waitForResult(ros::Duration(5));
        
        // Define Goal 
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = tf::resolve(g_tf_prefix, "base_link");
        goal.target_pose.header.stamp = ros::Time::now();
        // random move 
        double th_rand = ( rand() % 314 ) / 100.0;
        goal.target_pose.pose.position.x = cos(th_rand);
        goal.target_pose.pose.position.y = sin(th_rand);
        goal.target_pose.pose.orientation.w = 1.0;
        
        std::cout << goal.target_pose.header.frame_id << std::endl;

        ac->sendGoal(goal);
        // Wait for the action to return
        ac->waitForResult(ros::Duration(5));
        
        std::cout << "robot " << robot_id_ << " random move" << std::endl;
        
        // 重新选择目标节点 
        // 确定机器人当前所在的节点（距离机器人当前位置最近的节点） 
        current_vertex = identify_vertex(x_robot, y_robot);
        
        double current_time = ros::Time::now().toSec();
        
        // 更新地图中各节点的空闲率信息，由于未到达目标点，因此不更新全局变量 g_last_visit，仅更新类的成员变量 last_visit,instantaneous_idleness 
        {
            boost::mutex::scoped_lock idleness_lock(g_idleness_mutex);
            for(int i=0; i<dimension; ++i)
            {
                last_visit[i] = g_last_visit[i];
                instantaneous_idleness[i] = current_time - last_visit[i];
            }
        }
        
        // 决策发布下一个任务 
//        next_vertex = decide_next_vertex(x_robot, y_robot);
        next_vertex = random(current_vertex, vertex_web);
        
        // 发布 intention 更新消息 
        publish_intention(next_vertex, ros::Time::now().toSec(), x_robot, y_robot);
        
        send_goal(next_vertex, x_robot, y_robot);
    }
}


// 任务开始执行时，调用 active 函数 (仅一次)，可用于执行任务的初始化工作 
void TaskExecution::goalActiveCallback()
{
//  printf("%s: goal active callback \n", move_base_str_.c_str());

    // check whether the robot is stuck or not every 20 seconds 
    // initialization 
    check_time = ros::Time::now().toSec() + 20;
    // record the position of this robot 
    {
        boost::mutex::scoped_lock status_lock(g_status_mutex);
        check_x_robot = g_x_robot;
        check_y_robot = g_y_robot;
    }
}


// 任务执行过程中，以一定频率调用 feedback 函数 (频率快于1Hz)，可用于执行任务过程中的实时控制 
void TaskExecution::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
//  printf("%s: goal feedback callback \n", move_base_str_.c_str());

    // check whether the robot is stuck or not every 20 seconds 
    if( ros::Time::now().toSec() > check_time )
    {
        // get the position of this robot 
        double x_robot, y_robot, th_robot;
        {
            boost::mutex::scoped_lock status_lock(g_status_mutex);
            x_robot = g_x_robot;
            y_robot = g_y_robot;
            th_robot = g_th_robot;
        }
        
        // if the robot moves less than 1 meter in 20 seconds, it may be stuck in place. 
        double distance_move = sqrt(pow((check_x_robot - x_robot), 2) + pow((check_y_robot - y_robot), 2));
        if( distance_move < 1 )
        {
            // try to move from stuck 
            std::cout << "check robot " << robot_id_ << " is stuck at time " << check_time << std::endl;
            
            // 取消正在执行的任务 
//            ac->cancelAllGoals();
            // Wait for the action to return 
//            ac->waitForResult(ros::Duration(5));
            
            // Define Goal 
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = tf::resolve(g_tf_prefix, "base_link");
            goal.target_pose.header.stamp = ros::Time::now();
            // random move 
            double th_rand = ( rand() % 314 ) / 100.0;
            goal.target_pose.pose.position.x = cos(th_rand);
            goal.target_pose.pose.position.y = sin(th_rand);
            goal.target_pose.pose.orientation.w = 1.0;
            
            ac->sendGoal(goal);
            // Wait for the action to return
            ac->waitForResult(ros::Duration(5));
            
            std::cout << "robot " << robot_id_ << " random move" << std::endl;
            
            // 重新选择目标节点 
            // 确定机器人当前所在的节点（距离机器人当前位置最近的节点） 
            current_vertex = identify_vertex(x_robot, y_robot);
            
            double current_time = ros::Time::now().toSec();
            
            // 更新地图中各节点的空闲率信息，由于未到达目标点，因此不更新全局变量 g_last_visit，仅更新类的成员变量 last_visit,instantaneous_idleness 
            {
                boost::mutex::scoped_lock idleness_lock(g_idleness_mutex);
                for(int i=0; i<dimension; ++i)
                {
                    last_visit[i] = g_last_visit[i];
                    instantaneous_idleness[i] = current_time - last_visit[i];
                }
            }
            
            // 决策发布下一个任务 
//            next_vertex = decide_next_vertex(x_robot, y_robot);
            next_vertex = random(current_vertex, vertex_web);
            
            // 发布 intention 更新消息 
            publish_intention(next_vertex, ros::Time::now().toSec(), x_robot, y_robot);
            
            send_goal(next_vertex, x_robot, y_robot);
            
            // 返回 
            return;
        }
        
        // begin new record 
        check_time = ros::Time::now().toSec() + 20;
        {
            boost::mutex::scoped_lock status_lock(g_status_mutex);
            check_x_robot = g_x_robot;
            check_y_robot = g_y_robot;
        }
    }
}


// 决策发布下一个任务 
unsigned int TaskExecution::decide_next_vertex(double x_robot, double y_robot)
{
//    unsigned int next_vertex_local = random(current_vertex, vertex_web);
//    unsigned int next_vertex_local = conscientious_reactive(current_vertex, vertex_web, instantaneous_idleness);
//    unsigned int next_vertex_local = heuristic_conscientious_reactive(current_vertex, vertex_web, instantaneous_idleness);

  /*
    // heuristic_pathfinder_conscientious_cognitive 
    unsigned int next_vertex_local;
    if( (elem_s_path == 0) || (i_path == (elem_s_path-1)) )
    {
        // 重新规划路径 
        elem_s_path = heuristic_pathfinder_conscientious_cognitive(current_vertex, vertex_web, instantaneous_idleness, dimension, path);
        i_path = 1;
    }
    else
    {
        ++i_path;
    }
    next_vertex_local = path[i_path];
    
    std::cout << "robot_" << robot_id_ << " : HPCC plan  (target vertex: " << path[elem_s_path-1] << ") (" << i_path << "/" << elem_s_path << ")" << std::endl;

  */


//  /*
// expected_reactive 
    // 更新地图中所有节点的估计被访问时间(estimate_last_visit)，如果某节点被某机器人选为目标点则为机器人估计的到达时间，否则为上次被访问时间 
    double current_time = ros::Time::now().toSec();
    for(int i=0; i<dimension; ++i)
    {
        estimate_last_visit[i] = last_visit[i];
        {
            boost::mutex::scoped_lock intention_lock(g_intention_mutex);
            // get iterator positioned on the first element 
            std::map<int, int>::const_iterator map_it = g_intention_mate.begin();
            // for each element in the map 
            while (map_it != g_intention_mate.end())
            {
                // 如果当前节点 i 被某机器人选为目标点 
                if (i == map_it->second)
                {
                    // 检查map对象中某键是否存在 
                    if (g_intention_estimate_time.count(map_it->first))
                    {
                        if( ( (estimate_last_visit[i] < current_time) || (g_intention_estimate_time[map_it->first] < estimate_last_visit[i]) ) && (g_intention_estimate_time[map_it->first] > current_time) )
                        {
                            estimate_last_visit[i] = g_intention_estimate_time[map_it->first];
                        }
                    }
                }
                ++map_it;  // increment iterator to denote the next element 
            }
        }
    }
    // 调用函数计算下一个目标点 
    unsigned int next_vertex_local = expected_reactive (current_vertex, vertex_web, ros::Time::now().toSec(), x_robot, y_robot, vel_robot, estimate_last_visit, edge_avg);
    
//    unsigned int next_vertex_local = mixed_reactive (current_vertex, vertex_web, ros::Time::now().toSec(), x_robot, y_robot, vel_robot, estimate_last_visit, edge_avg, dimension);

//  */


  /*
// expected_cognitive 
    // 更新地图中所有节点的估计被访问时间(estimate_last_visit)，如果某节点被某机器人选为目标点则为机器人估计的到达时间，否则为上次被访问时间 
    double current_time = ros::Time::now().toSec();
    for(int i=0; i<dimension; ++i)
    {
        estimate_last_visit[i] = last_visit[i];
        // 考虑其他机器人的长期目标点  goal_mate 
        {
            boost::mutex::scoped_lock goal_lock(g_goal_mutex);
            // get iterator positioned on the first element 
            std::map<int, int>::const_iterator map_it = g_goal_mate.begin();
            // for each element in the map 
            while (map_it != g_goal_mate.end())
            {
                // 如果当前节点 i 被某机器人选为目标点 
                if (i == map_it->second)
                {
                    // 检查map对象中某键是否存在 
                    if (g_goal_estimate_time.count(map_it->first))
                    {
                        if( ( (estimate_last_visit[i] < current_time) || (g_goal_estimate_time[map_it->first] < estimate_last_visit[i]) ) && (g_goal_estimate_time[map_it->first] > current_time) )
                        {
                            estimate_last_visit[i] = g_goal_estimate_time[map_it->first];
                        }
                    }
                }
                ++map_it;  // increment iterator to denote the next element 
            }
        }
        // 考虑其他机器人的下一个目标点  intention_mate 
        {
            boost::mutex::scoped_lock intention_lock(g_intention_mutex);
            // get iterator positioned on the first element 
            std::map<int, int>::const_iterator map_it = g_intention_mate.begin();
            // for each element in the map 
            while (map_it != g_intention_mate.end())
            {
                // 如果当前节点 i 被某机器人选为目标点 
                if (i == map_it->second)
                {
                    // 检查map对象中某键是否存在 
                    if (g_intention_estimate_time.count(map_it->first))
                    {
                        if( ( (estimate_last_visit[i] < current_time) || (g_intention_estimate_time[map_it->first] < estimate_last_visit[i]) ) && (g_intention_estimate_time[map_it->first] > current_time) )
                        {
                            estimate_last_visit[i] = g_intention_estimate_time[map_it->first];
                        }
                    }
                }
                ++map_it;  // increment iterator to denote the next element 
            }
        }
    }
    // 调用函数计算下一个目标点 
    unsigned int goal_vertex;
    double goal_time;
    
//    unsigned int next_vertex_local = expected_cognitive (current_vertex, vertex_web, ros::Time::now().toSec(), x_robot, y_robot, vel_robot, estimate_last_visit, edge_avg, dimension, instantaneous_idleness, goal_vertex, goal_time);
    // 注意：该函数可能会改变变量 instantaneous_idleness 中的某些值，只要在之后不使用即可(仅限于此次执行 decide_next_vertex 函数中)。 每次执行 decide_next_vertex 函数前都会重新计算 instantaneous_idleness 等变量的值 
    
    unsigned int next_vertex_local = expected_cognitive_around (current_vertex, vertex_web, ros::Time::now().toSec(), x_robot, y_robot, vel_robot, estimate_last_visit, edge_avg, dimension, instantaneous_idleness, goal_vertex, goal_time);
    
    std::cout << "robot_" << robot_id_ << " : EC plan  (target vertex: " << goal_vertex << ") (next vertex: " << next_vertex_local << ")" << std::endl;
    
    // 发布 goal 更新消息 
    lcm::LCM lcm;
    if(!lcm.good())
        exit(0);
    exlcm::goal_t goal_msg;
    goal_msg.robot_id = robot_id_;
    goal_msg.vertex_id = goal_vertex;
    goal_msg.decision_time = ros::Time::now().toSec();
    goal_msg.estimate_time = goal_time;
    lcm.publish("VERTEXGOAL", &goal_msg);

  */


  /*
// SEBS (state_exchange_bayesian_strategy) or GBS (greedy_bayesian_strategy) 
    double G1 = 0.1;
    
//    double G2 = 20.54;  // grid, 1 robots 
//    double G2 = 17.70;  // grid, 2 robots 
//    double G2 = 11.15;  // grid, 4 robots 
//    double G2 = 10.71;  // grid, 6 robots 
//    double G2 = 10.29;  // grid, 8 robots 
//    double G2 = 9.13;  // grid, 12 robots 
    
//    double G2 = 152.0;  // cumberland, 1 robots 
//    double G2 = 100.4;  // cumberland, 2 robots 
    double G2 = 80.74;  // cumberland, 4 robots 
//    double G2 = 77.0;  // cumberland, 6 robots 
//    double G2 = 63.5;  // cumberland, 8 or 12 robots 
    
//    double edge_min = 1.0;  // grid 
    double edge_min = 50.0;  // cumberland 
    
    int nr_robots;
    int *tab_intention;
    // 更新其他机器人的目标点信息 
    {
        boost::mutex::scoped_lock intention_lock(g_intention_mutex);
        nr_robots = g_intention_mate.size();
        tab_intention = new int[nr_robots];
        int i=0;
        // get iterator positioned on the first element 
        std::map<int, int>::const_iterator map_it = g_intention_mate.begin();
        // for each element in the map 
        while (map_it != g_intention_mate.end())
        {
            tab_intention[i] = map_it->second;
            ++i;
            ++map_it;  // increment iterator to denote the next element 
        }
    }
    unsigned int next_vertex_local = state_exchange_bayesian_strategy(current_vertex, vertex_web, instantaneous_idleness, tab_intention, nr_robots, G1, G2, edge_min);
    delete [] tab_intention;
    
    // use GBS instead of SEBS 
//    next_vertex_local = greedy_bayesian_strategy(current_vertex, vertex_web, instantaneous_idleness, G1, G2, edge_min);

  */


  /*
// centralized partion-based algorithm 
// 分区巡逻算法，按部就班地一步步执行，不会中途调整巡逻计划 
    ++plan_vertex_i;
    if( plan_vertex_i == plan_vertex_num )  plan_vertex_i = 0;
    unsigned int next_vertex_local = plan_vertex_list[plan_vertex_i];
    
    std::cout << "robot_" << robot_id_ << " : partition plan  (" << plan_vertex_i << "/" << plan_vertex_num << ")" << std::endl;

  */
    
    return next_vertex_local;
}


// 发布 idleness 更新消息 
void TaskExecution::publish_idleness(int vertex_id, double current_time)
{
    // 发布 idleness 更新消息 
    lcm::LCM lcm;
    if(!lcm.good())
        exit(0);
    
    exlcm::idleness_t idleness_msg;
    
    idleness_msg.robot_id = robot_id_;
    idleness_msg.vertex_id = vertex_id;
    idleness_msg.visit_time = current_time;
    
    lcm.publish("VERTEXIDLENESS", &idleness_msg);
}


// 发布 intention 更新消息 
void TaskExecution::publish_intention(int vertex_id, double current_time, double x_robot, double y_robot)
{
    // 估计到达目标所需的时间 
    double x_target = ( vertex_web[next_vertex].x ) * RESOLUTION;
//    double y_target = ( HEIGHT_PX - vertex_web[next_vertex].y ) * RESOLUTION;
    double y_target = ( vertex_web[next_vertex].y ) * RESOLUTION;
    
    double distance_linear = sqrt(pow((x_target - x_robot), 2) + pow((y_target - y_robot), 2));  // linear distance 
    double estimate_time = current_time + distance_linear / vel_robot;  // 对到达目标的时间的估计 
    
    // 发布 intention 更新消息 
    lcm::LCM lcm;
    if(!lcm.good())
        exit(0);
    
    exlcm::intention_t intention_msg;
    
    intention_msg.robot_id = robot_id_;
    intention_msg.vertex_id = vertex_id;
    intention_msg.decision_time = current_time;
    intention_msg.estimate_time = estimate_time;
    
    lcm.publish("VERTEXINTENTION", &intention_msg);
}


// 确定机器人当前所在的节点（距离机器人当前位置最近的节点） 
unsigned int TaskExecution::identify_vertex(double x_robot, double y_robot)
{
    unsigned int index_min = 0;
    double x_vertex = ( vertex_web[index_min].x ) * RESOLUTION;
//    double y_vertex = ( HEIGHT_PX - vertex_web[index_min].y ) * RESOLUTION;
    double y_vertex = ( vertex_web[index_min].y ) * RESOLUTION;
    double distance_min = (x_robot-x_vertex)*(x_robot-x_vertex)+(y_robot-y_vertex)*(y_robot-y_vertex);
    for(int i=1; i<dimension; ++i)
    {
        x_vertex = ( vertex_web[i].x ) * RESOLUTION;
//        y_vertex = ( HEIGHT_PX - vertex_web[i].y ) * RESOLUTION;
        y_vertex = ( vertex_web[i].y ) * RESOLUTION;
        double distance_this = (x_robot-x_vertex)*(x_robot-x_vertex)+(y_robot-y_vertex)*(y_robot-y_vertex);
        if( distance_this < distance_min )
        {
            index_min = i;
            distance_min = distance_this;
        }
    }
    
    return index_min;
}


// 发布到达目标节点的任务 
void TaskExecution::send_goal(unsigned int next_vertex, double x_robot, double y_robot)
{
    double x_target = ( vertex_web[next_vertex].x ) * RESOLUTION;
//    double y_target = ( HEIGHT_PX - vertex_web[next_vertex].y ) * RESOLUTION;
    double y_target = ( vertex_web[next_vertex].y ) * RESOLUTION;
    
    double radians = atan2(y_target-y_robot, x_target-x_robot);  // 角度 
    
    // Define Goal 
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x_target;
    goal.target_pose.pose.position.y = y_target;
    
    tf::Quaternion quat_tf;
    quat_tf = tf::createQuaternionFromYaw(radians);
    geometry_msgs::Quaternion quat_ge;
    tf::quaternionTFToMsg(quat_tf, quat_ge);
    
//  geometry_msgs::Quaternion quat_ge = tf::createQuaternionMsgFromYaw(radians);
    
    goal.target_pose.pose.orientation = quat_ge;
    
    ac->sendGoal(goal, boost::bind(&TaskExecution::goalDoneCallback, this, _1, _2), boost::bind(&TaskExecution::goalActiveCallback,this), boost::bind(&TaskExecution::goalFeedbackCallback, this, _1));
    // Wait for the action to return
    ac->waitForResult(ros::Duration(5));
}
