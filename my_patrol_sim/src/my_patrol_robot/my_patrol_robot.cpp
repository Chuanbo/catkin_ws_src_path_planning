#include <ros/ros.h>

#include "global_variables.h"
#include "task_execution.h"
#include "message_listener.h"

#include <lcm/lcm-cpp.hpp>
#include "exlcm/status_t.hpp"


class MyRobot
{
public:
    MyRobot() : 
        nh_(),
        timer_status_publish_(nh_.createTimer(ros::Duration(1.0), &MyRobot::timerStatusPublish_callback, this)) 
    {
        robot_id_ = g_robot_id;
        
//        ros::param::get("~tf_prefix", g_tf_prefix);  // private parameter 
        if ( ros::param::get("~tf_prefix", g_tf_prefix) ) {
            std::cout << "Received tf_prefix parameter.  Stage Simulation" << std::endl;
        }
        else {
            std::cout << "No tf_prefix parameter.  TurtleBot Experiment" << std::endl;
        }
        
        robot_frame_ = tf::resolve(g_tf_prefix, "base_footprint");
        
//        std::cout << g_tf_prefix << std::endl;
        
        if ( ros::param::get("~vel_robot", g_vel_robot) ) {
            std::cout << "g_vel_robot = " << g_vel_robot << std::endl;
        }
        else {
            g_vel_robot = 0.2;  // default 
            std::cout << "g_vel_robot = " << g_vel_robot << std::endl;
        }
        
        listener_.waitForTransform("/map", robot_frame_, ros::Time(0), ros::Duration(10.0));
    }
    
    ~MyRobot() 
    {}
    
    void timerStatusPublish_callback(const ros::TimerEvent& event);
    
    ros::NodeHandle nh_;
    
    ros::Timer timer_status_publish_;
    
    int robot_id_;
    
    std::string robot_frame_;
    
    tf::TransformListener listener_;
};


void MyRobot::timerStatusPublish_callback(const ros::TimerEvent& event)
{
    double x_robot, y_robot, th_robot; // 机器人当前的位姿 
    // 通过tf获取机器人当前的位姿 
    tf::StampedTransform transform;
    try {
        listener_.lookupTransform("/map", robot_frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    x_robot = transform.getOrigin().x();
    y_robot = transform.getOrigin().y();
    // tf::Quaternion 
    tf::Quaternion quat = transform.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);
    th_robot = yaw;  // Yaw around Z axis
    
    {
        boost::mutex::scoped_lock status_lock(g_status_mutex);
        g_x_robot = x_robot;
        g_y_robot = y_robot;
        g_th_robot = th_robot;
    }
    
    // 机器人发布自身位置信息 
    lcm::LCM lcm;
    if(!lcm.good())
        exit(0);

    exlcm::status_t status_msg;
    
    status_msg.robot_id = robot_id_;
    status_msg.frame_id = robot_frame_;
    status_msg.x_robot = x_robot;
    status_msg.y_robot = y_robot;
    status_msg.th_robot = th_robot;

    lcm.publish("ROBOTSTATUS", &status_msg);
}


int main(int argc, char** argv)
{
    if (argc < 2) {
        ROS_ERROR("You must specify robot id.");
        return -1;
    }
    if (argc < 3) {
        ROS_ERROR("You must specify graph file.");
        return -1;
    }
    
    char *id = argv[1];
    char *graph_name = argv[2];
    
    // 全局变量 g_robot_id 
    g_robot_id = atoi(id);
    
    // std::cout << "robot_" << g_robot_id << std::endl;
    
    std::string ros_node_name = "robot_" + std::to_string(g_robot_id);

    std::cout << ros_node_name << std::endl;
    
    ros::init( argc, argv, ros_node_name );
    ros::Time::init();
    
    MyRobot my_robot;
    
    // 全局变量 g_graph_name 
    g_graph_name = graph_name;
    unsigned int dimension = GetGraphDimension( g_graph_name.c_str() );
    
    std::cout << g_graph_name << std::endl;
    
    // 初始化地图中各节点的空闲率信息 
    g_last_visit = new double[dimension];
    for(int i=0; i<dimension; ++i)
    {
        g_last_visit[i] = 0.0;
    }
    
    // 启动消息处理线程 
    boost::thread thrd_message(message_thread);
    
    // 启动任务执行线程 
    boost::thread thrd_execution(execution_thread);
    
    ros::spin();
    
    // 在主线程退出前，中断子线程 
    thrd_message.interrupt();
    thrd_execution.interrupt();
    
    delete [] g_last_visit;
    
    return 0;
}
