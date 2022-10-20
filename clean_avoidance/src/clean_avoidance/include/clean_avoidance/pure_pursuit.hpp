#include <string>
#include <cmath>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>  //visual mark
#include <kdl/frames.hpp>

class PurePursuit
{
public:

  //! Constructor
  PurePursuit();

  void PublishVisual(geometry_msgs::TransformStamped& current_ind);
  void PublishVisual(geometry_msgs::PoseStamped& current_pose);

  void ComputeVelocities();

  bool PathInit();

  void ObstacleAvoidance(const sensor_msgs::LaserScanConstPtr& laser);

  bool CheckFrontObstacle(const sensor_msgs::LaserScanConstPtr& laser);

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame TransformToBaseLink(const geometry_msgs::Pose& pose,
                                 const geometry_msgs::Transform& tf);

  template<typename T1, typename T2>
  double Distance(T1& pt1, T2& pt2)
  {
    //根号下x的平方+y的平方+z的平方
    //return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) );
  }

  //! Run the controller.
  void Run();

  bool goal_reached_;  //到达目标判断
  bool interrupt_request_ = false;  //中断请求标志位
  int idx_;       //跟踪index

  bool ResetID(){
      idx_ = 0;
  }

  bool ResetPath(){
      path_ = NULL;
  }
  
  bool ClearVisualPath();

  nav_msgs::Path* path_;

private:
  // Vehicle parameters：车辆参数
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  //位置公差沿机器人的x轴测量
  double ld_, pos_tol_;
  // Generic control variables：控制变量
  double v_max_, v_, w_max_;

  // Steering angle is denoted by delta：转向角用delta表示
  double delta_, delta_vel_, acc_, jerk_, delta_max_;

  double x_region_;    //激光雷达检测范围x
  double y_region_;    //激光雷达检测范围y
  int offset_;                   //激光雷达开始检测的线束
  bool check_front_obstacle_ = false;

  geometry_msgs::Twist cmd_vel_;
  
  // Ros infrastructure
  ros::NodeHandle nh_; 
  ros::NodeHandle nh_private_;
  //ros::Subscriber sub_odom_;
  ros::Publisher pub_vel_;   //这里主要按照差速：pub_vel
  ros::Publisher setpoint_pub_;           //mark point pub
  ros::Subscriber laser_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  std::string map_frame_id_; 
  std::string robot_frame_id_; 
  std::string lookahead_frame_id_; 
};