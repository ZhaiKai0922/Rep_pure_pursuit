#include <vector>
#include <string>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <actionlib/server/simple_action_server.h>
#include <clean_avoidance/CleanAvoidanceAction.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <pluginlib/class_loader.hpp>
#include <math.h>
#include <vector>

#include "clean_avoidance/pure_pursuit.hpp"   //纯跟踪

#include "nav_core/base_global_planner.h"
#include <pluginlib/class_loader.hpp>

using namespace std;

class CleanAvoidance{
    public:
    CleanAvoidance(tf2_ros::Buffer& tf);
    ~CleanAvoidance();
    void ExecuteCb(const clean_avoidance::CleanAvoidanceGoalConstPtr& clean_goal);  //action回调函数，接收目标点
    void MakeLine(const vector<double>& first, const vector<double>& second);    //两点生成直线
    bool PathInit(const vector<vector<double>>& goals);
    bool FixPath();     //根据costmap实时修改直线
    bool CycleFixPath();

    void LineVisual();
    void MergeAvoidWaypoints(const vector<geometry_msgs::PoseStamped>& path, int& end_of_avoid_index);
    void ResetPath();
    void PubZeroVel();
    bool FindMinIndex(const int& start, const int& end, int& temp_index);
    float ReadTaskPercentage();

    void PubVelThread();
    bool Init(vector<vector<double>> points);
    bool Run();
    bool Start();
    bool Stop();
    bool Cancel();

    tf2_ros::Buffer& tf_;
    ros::Publisher vel_pub_;
    ros::Publisher path_pub_;

    actionlib::SimpleActionServer<clean_avoidance::CleanAvoidanceAction>* as_;

    costmap_2d::Costmap2DROS* planner_costmap_ros_;

    bool pause_voice_flag = false; //停车语音标志位，每当中断pure_pursuit则停车
    bool goal_reach = false;
    bool stop_flag = false;
    bool cancel_flag = false;

    private:
    //全局规划器
    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
    //以插件形式实现全局规划器、局部规划器和丢失时恢复规划器。
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

    boost::shared_ptr<PurePursuit> pure_pursuit_;
    boost::shared_ptr<boost::thread> pub_vel_thread_;

    costmap_2d::Costmap2D* my_costmap_;

    nav_msgs::Path path_;
    bool avoid_flag_ = false;
    bool fix_path_ = false;

    bool pub_vel_thread_flag_ = false;              //纯路径跟踪线程标志位
    bool path_init_flag_ = false;                            //路径初始化标志位

    int cycle_start_index_ = 0;                                                //遍历路径的初始点
    int cycle_end_index_ = 0;                                                 //遍历路径的结束点
    int start_waypoint_index_ = -1;                     //路径规划初始点
    int goal_waypoint_index_ = -1;                      //路径规划结束点
    int end_of_avoid_index = -1;                           //路径规划结束点

    int first_search_waypoints_delta_ = 30;     //
    int search_waypoints_delta_ = 15;       //
    int search_waypoints_size_ = 76;         //
    double path_resolution_ = 0.1;
    double safe_distance_ = 0.2;
    int reduce_start_points_ = 5;
    int number_greater_index_ = 10;
    int number_less_index_ = 10;
    int cycle_reduce_start_points_ = 3;


    vector<double> robot_pose_{0.0, 0.0};
};