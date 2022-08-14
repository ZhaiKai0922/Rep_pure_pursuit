#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>  //visual mark
#include <kdl/frames.hpp>
#include </home/zk/zk/pure_pursuit/cmd_pursuit_ws/src/double_pure_pursuit/include/pub_path.hpp>

class PurePursuit
{
public:

  //! Constructor
  PurePursuit(std::vector<std::vector<double>> path_points);

  void PublishVisual(geometry_msgs::TransformStamped& current_ind);
  void PublishVisual(geometry_msgs::PoseStamped& current_pose);

  //! Compute velocit commands each time new odometry data is received.
  void ComputeVelocities(const nav_msgs::OdometryConstPtr& odom);

  //! Receive path to follow.
  void ReceivePath(nav_msgs::Path& path);

  void PathInit();

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame TransformToBaseLink(const geometry_msgs::Pose& pose,
                                 const geometry_msgs::Transform& tf);

  template<typename T1, typename T2>
  double Distance(T1& pt1, T2& pt2)
  {
    //根号下x的平方+y的平方+z的平方
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }

  //! Run the controller.
  void Run();
  
private:
  
  PubPath pub_path_;   //通过点生成直线并发布path

  // Vehicle parameters：车辆参数
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  //位置公差沿机器人的x轴测量
  double ld_, pos_tol_;
  // Generic control variables：控制变量
  double v_max_, v_, w_max_;
  // Control variables for Ackermann steering：阿克曼底盘
  // Steering angle is denoted by delta：转向角用delta表示
  double delta_, delta_vel_, acc_, jerk_, delta_max_;
  nav_msgs::Path path_;
  unsigned idx_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  ackermann_msgs::AckermannDriveStamped cmd_acker_;
  
  // Ros infrastructure
  ros::NodeHandle nh_; 
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_path_;
  ros::Publisher pub_vel_;   //这里主要按照差速：pub_vel
  ros::Publisher pub_acker_;
  ros::Publisher setpoint_pub_;           //mark point pub
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  std::string map_frame_id_; 
  std::string robot_frame_id_; 
  std::string lookahead_frame_id_; 
  std::string acker_frame_id_;
};