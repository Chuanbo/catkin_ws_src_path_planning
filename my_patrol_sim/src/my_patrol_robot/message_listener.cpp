#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "global_variables.h"

#include "message_listener.h"

#include <cmath>
#include <iostream>


// 消息处理线程函数 
void message_thread()
{
    lcm::LCM lcm;

    if(!lcm.good())
        exit(0);

    Handler handlerObject;
    
    lcm.subscribe("ROBOTCOMMAND", &Handler::handleMessageCommand, &handlerObject);
    
    lcm.subscribe("VERTEXIDLENESS", &Handler::handleMessageIdleness, &handlerObject);
    
    lcm.subscribe("VERTEXINTENTION", &Handler::handleMessageIntention, &handlerObject);
    
    lcm.subscribe("VERTEXGOAL", &Handler::handleMessageGoal, &handlerObject);
    
    g_plan_vertex_num = 0;  // 初始化 plan，以该变量是否为0，来判断是否存在多步计划 
    
    lcm.subscribe("PLANVERTEXLIST", &Handler::handleMessagePlan, &handlerObject);
    
    lcm.subscribe("ROBOTSTATUS", &Handler::handleMessageState, &handlerObject);

    while(0 == lcm.handle());
}


void Handler::handleMessageCommand(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const exlcm::command_t* command_msg)
{
    if ( command_msg->is_start == true && command_msg->is_stop == false )
    {
        g_is_start = true;
        g_cond_start.notify_one();
    }
    else if ( command_msg->is_start == false && command_msg->is_stop == true )
    {
        g_is_stop = true;
        g_cond_stop.notify_one();
    }
    else
    {
        printf("INACCURATE ROBOT COMMAND \n");
    }
}


void Handler::handleMessageIdleness(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const exlcm::idleness_t* idleness_msg)
{
    if( idleness_msg->robot_id == robot_id_ )  return;
    
    // 建模通讯的可靠性 
    if( ( rand() % 100 ) > com_rate_ )  return;
    
    // 建模通讯的距离限制 
    if( robot_set_.count(idleness_msg->robot_id) == 0 )
    {
      /*
        std::set<int>::iterator it_set;
        std::cout << "Robot_" << robot_id_ << " give up message from Robot_" << idleness_msg->robot_id << " because out of range." << std::endl;
        std::cout << "Robot_" << robot_id_ << " around robot_id: ";
        for(it_set = robot_set_.begin(); it_set != robot_set_.end(); ++it_set)
        {
            std::cout << *it_set << " ";
        }
        std::cout << std::endl;
      */
        // 返回 
        return;
    }
    
    int vertex_id = idleness_msg->vertex_id;
    double visit_time = idleness_msg->visit_time;
    
    
    // 建模通讯的时间延迟 
    // 通讯时延大于1秒时才予以考虑 
    if( com_delay_ > 1 )
    {
        idleness_pub one_idleness_pub;
        one_idleness_pub.vertex_id = vertex_id;
        one_idleness_pub.visit_time = visit_time;
        one_idleness_pub.update_time = visit_time + com_delay_;
        
        idleness_pub_list.push_back(one_idleness_pub);
        // 返回 
        return;
    }
    
    
    // 更新地图中节点的空闲率信息 
    {
        boost::mutex::scoped_lock idleness_lock(g_idleness_mutex);
        if( visit_time > g_last_visit[vertex_id] )
        {
            g_last_visit[vertex_id] = visit_time;
        }
    }
}


void Handler::handleMessageIntention(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::intention_t* intention_msg)
{
    if( intention_msg->robot_id == robot_id_ )  return;
    
    // 建模通讯的可靠性 
    if( ( rand() % 100 ) > com_rate_ )  return;
    
    // 建模通讯的距离限制 
    if( robot_set_.count(intention_msg->robot_id) == 0 )
    {
      /*
        std::set<int>::iterator it_set;
        std::cout << "Robot_" << robot_id_ << " give up message from Robot_" << intention_msg->robot_id << " because out of range." << std::endl;
        std::cout << "Robot_" << robot_id_ << " around robot_id: ";
        for(it_set = robot_set_.begin(); it_set != robot_set_.end(); ++it_set)
        {
            std::cout << *it_set << " ";
        }
        std::cout << std::endl;
      */
        // 返回 
        return;
    }
    
    int robot_id = intention_msg->robot_id;
    int vertex_id = intention_msg->vertex_id;
    double decision_time = intention_msg->decision_time;
    double estimate_time = intention_msg->estimate_time;
    
    
    // 建模通讯的时间延迟 
    // 通讯时延大于1秒时才予以考虑 
    if( com_delay_ > 1 )
    {
        intention_pub one_intention_pub;
        one_intention_pub.robot_id = robot_id;
        one_intention_pub.vertex_id = vertex_id;
        one_intention_pub.decision_time = decision_time;
        one_intention_pub.estimate_time = estimate_time;
        one_intention_pub.update_time = decision_time + com_delay_;
        
        intention_pub_list.push_back(one_intention_pub);
        // 返回 
        return;
    }
    
    
    // 更新其他机器人要到达的下一个目标点信息 
    {
        boost::mutex::scoped_lock intention_lock(g_intention_mutex);
        g_intention_mate[robot_id] = vertex_id;
        g_intention_decision_time[robot_id] = decision_time;
        g_intention_estimate_time[robot_id] = estimate_time;
    }
}


void Handler::handleMessageGoal(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::goal_t* goal_msg)
{
    if( goal_msg->robot_id == robot_id_ )  return;
    
    // 建模通讯的可靠性 
    if( ( rand() % 100 ) > com_rate_ )  return;
    
    int robot_id = goal_msg->robot_id;
    int vertex_id = goal_msg->vertex_id;
    double decision_time = goal_msg->decision_time;
    double estimate_time = goal_msg->estimate_time;
    
    // 更新其他机器人规划要到达的目标点信息 
    {
        boost::mutex::scoped_lock goal_lock(g_goal_mutex);
        g_goal_mate[robot_id] = vertex_id;
        g_goal_decision_time[robot_id] = decision_time;
        g_goal_estimate_time[robot_id] = estimate_time;
    }
}


void Handler::handleMessagePlan(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::plan_t* plan_msg)
{
    if( plan_msg->robot_id != robot_id_ )  return;
    
    g_plan_vertex_num = plan_msg->num_step;
    g_plan_vertex_list.clear();
    for(int i=0; i<g_plan_vertex_num; ++i)
    {
        g_plan_vertex_list.push_back( plan_msg->steps[i] );
    }
    
    std::cout << "robot_" << robot_id_ << " : Received Partition Patrol Plan" << std::endl;
    std::cout << "Patrol Plan : ";
    for(int i=0; i<g_plan_vertex_num; ++i)
    {
        std::cout << " " << g_plan_vertex_list[i];
    }
    std::cout << std::endl;
}


void Handler::handleMessageState(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::status_t* status_msg)
{
    // 检查是否有消息等待发布 
    if( status_msg->robot_id == robot_id_ )
    {
        if( idleness_pub_list.empty() && intention_pub_list.empty() )  return;
        
        lcm::LCM lcm;
        if(!lcm.good())
            exit(0);
        
        double current_time = ros::Time::now().toSec();
        
        while( !idleness_pub_list.empty() )
        {
            idleness_pub one_idleness_pub;
            one_idleness_pub = idleness_pub_list[0];
            
            if( one_idleness_pub.update_time > (current_time + 1) )  break;
                        
            // 更新地图中节点的空闲率信息 
            {
                boost::mutex::scoped_lock idleness_lock(g_idleness_mutex);
                if( one_idleness_pub.visit_time > g_last_visit[one_idleness_pub.vertex_id] )
                {
                    g_last_visit[one_idleness_pub.vertex_id] = one_idleness_pub.visit_time;
                }
            }
            
            idleness_pub_list.erase(idleness_pub_list.begin());  // 删除已更新的消息 
  /*
            // 当此机器人是 robot_0 时，输出消息延迟信息，用于调试需要 
            if ( robot_id_ == 0 )
            {
                std::cout << "visit_time: " << one_idleness_pub.visit_time << std::endl;
                std::cout << "delay_time: " << current_time << std::endl;
            }
  */
        }
        
        while( !intention_pub_list.empty() )
        {
            intention_pub one_intention_pub;
            one_intention_pub = intention_pub_list[0];
            
            if( one_intention_pub.update_time > (current_time + 1) )  break;
            
            // 更新其他机器人要到达的下一个目标点信息 
            {
                boost::mutex::scoped_lock intention_lock(g_intention_mutex);
                g_intention_mate[one_intention_pub.robot_id] = one_intention_pub.vertex_id;
                g_intention_decision_time[one_intention_pub.robot_id] = one_intention_pub.decision_time;
                g_intention_estimate_time[one_intention_pub.robot_id] = one_intention_pub.estimate_time;
            }
            
            intention_pub_list.erase(intention_pub_list.begin());  // 删除已更新的消息 
  /*
            // 当此机器人是 robot_0 时，输出消息延迟信息，用于调试需要 
            if ( robot_id_ == 0 )
            {
                std::cout << "decision_time: " << one_intention_pub.decision_time << std::endl;
                std::cout << "delay_time: " << current_time << std::endl;
            }
  */
        }
    
        // 返回 
        return;
    }
    
    // 机器人当前的位姿 
    double x_robot, y_robot, th_robot;
    {
        boost::mutex::scoped_lock status_lock(g_status_mutex);
        x_robot = g_x_robot;
        y_robot = g_y_robot;
        th_robot = g_th_robot;
    }
    
    double distance_linear = sqrt(pow((status_msg->x_robot - x_robot), 2) + pow((status_msg->y_robot - y_robot), 2));  // linear distance 
    if ( distance_linear < com_dist_ )
    {
        // 在通讯范围内，则加入机器人列表 
        robot_set_.insert(status_msg->robot_id);
    }
    else
    {
        // 若不在通讯范围内，则从机器人列表中删除 
        robot_set_.erase(status_msg->robot_id);
    }
    
  /*
    // 当此机器人是 robot_0 时，输出其机器人列表，用于调试需要 
    if ( robot_id_ == 0 )
    {
        std::set<int>::iterator it_set;
        std::cout << "Robot_0 around robot_id: ";
        for(it_set = robot_set_.begin(); it_set != robot_set_.end(); ++it_set)
        {
            std::cout << *it_set << " ";
        }
        std::cout << std::endl;
    }
  */
}
