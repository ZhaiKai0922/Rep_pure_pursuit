#include "clean_avoidance/pure_pursuit.hpp"

using namespace std;

PurePursuit::PurePursuit() : ld_(1.2), v_max_(0.5), v_(0.1), w_max_(1.0), pos_tol_(0.1), idx_(0),
                             goal_reached_(true), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("base_link"),
                             lookahead_frame_id_("lookahead")
{
  // Get parameters from the parameter server
  nh_private_.param<double>("wheelbase", L_, 1.0);                                                       //轴距
  nh_private_.param<double>("lookahead_distance", ld_, 1.2);                                 //前沿点距离
  nh_private_.param<double>("linear_velocity", v_, 0.3);                                              //线速度设定值
  nh_private_.param<double>("max_rotational_velocity", w_max_, 0.3);             //最大角速度
  nh_private_.param<double>("position_tolerance", pos_tol_, 0.1);                        //位置误差
  nh_private_.param<double>("steering_angle_velocity", delta_vel_, 100.0);     //转角速度
  nh_private_.param<double>("acceleration", acc_, 100.0);                                          //加速度
  nh_private_.param<double>("jerk", jerk_, 100.0);
  nh_private_.param<double>("steering_angle_limit", delta_max_, 1.57);            //转角最大
  nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "base_link");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");
  // Frame attached to midpoint of front axle (for front-steered vehicles).

  nh_private_.param<double>("x_region", x_region_, 0.6);                                           //x检测范围
  nh_private_.param<double>("y_region", y_region_, 0.5);                                           //y检测范围
  nh_private_.param<int>("offset", offset_, 250);                                                               //从offset开始检测

  // Populate messages with static data
  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;
  
  //sub_odom_ = nh_.subscribe("/odom", 1, &PurePursuit::ComputeVelocities, this);
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  setpoint_pub_ = nh_.advertise<visualization_msgs::Marker>("/set_point", 1);             //mark point pub
  laser_sub_ = nh_.subscribe("/scan", 10, &PurePursuit::ObstacleAvoidance, this);    //scan

  std::cout << "param linear_velocity = " << v_ << std::endl;
  std::cout << "param lookahead_distance = " << ld_ << std::endl;
  std::cout << "param x_region = " << x_region_ << std::endl;
  std::cout << "param y_region = " << y_region_ << std::endl;
  std::cout << "param offset = " << offset_ << std::endl;
}

//case：_current_ind
void PurePursuit::PublishVisual(geometry_msgs::TransformStamped& current_ind)
{
          visualization_msgs::Marker p;
          p.header.frame_id = "base_link";
          p.header.stamp = ros::Time::now();
          p.id = 1;

        // p.type = visualization_msgs::Marker::SPHERE;
        // p.action = visualization_msgs::Marker::ADD;

          p.pose.position.x = current_ind.transform.translation.x;
          p.pose.position.y = current_ind.transform.translation.y;
          p.pose.position.z = current_ind.transform.translation.z;
          p.pose.orientation.w = current_ind.transform.rotation.w;
          p.pose.orientation.x = current_ind.transform.rotation.x;
          p.pose.orientation.y = current_ind.transform.rotation.y;
          p.pose.orientation.z = current_ind.transform.rotation.z;

          p.scale.x = p.scale.y = p.scale.z = 0.2;

          p.color.a = p.color.r = 1.0;
          p.color.g = p.color.b = 0.0;

         setpoint_pub_.publish(p);
}

    void PurePursuit::PublishVisual(geometry_msgs::PoseStamped& current_pose)
{
          visualization_msgs::Marker p;
          p.header.frame_id = "map";
          p.header.stamp = ros::Time::now();
          p.id = 1;

        // p.type = visualization_msgs::Marker::SPHERE;
        // p.action = visualization_msgs::Marker::ADD;

          p.pose.position.x = current_pose.pose.position.x;
          p.pose.position.y = current_pose.pose.position.y;
          p.pose.position.z = current_pose.pose.position.z;
          p.pose.orientation.w =  current_pose.pose.orientation.w;
          p.pose.orientation.x = current_pose.pose.orientation.x;
          p.pose.orientation.y = current_pose.pose.orientation.y;
          p.pose.orientation.z = current_pose.pose.orientation.z;

          p.scale.x = p.scale.y = p.scale.z = 0.2;

          p.color.a = p.color.r = 1.0;
          p.color.g = p.color.b = 0.0;

         setpoint_pub_.publish(p);
}

//laser的回调函数
void PurePursuit::ObstacleAvoidance(const sensor_msgs::LaserScanConstPtr& laser)
{
  //std::cout << "===================laser callback !!!!======================" << std::endl;
  if(CheckFrontObstacle(laser)){
    check_front_obstacle_ = true;
  }else{
    check_front_obstacle_ = false;
  }
}

bool PurePursuit::CheckFrontObstacle(const sensor_msgs::LaserScanConstPtr& laser)
{
  //std::cout << "laser -> ranges.size()" << laser->ranges.size() << std::endl;

  for(int i = offset_; i < laser->ranges.size() - offset_; i++){
      double x = laser->ranges[i] * sin(laser->angle_max - i * laser->angle_increment);
      double y = laser->ranges[i] * cos(laser->angle_max - i * laser->angle_increment);

      if( 0 < fabs(x) && fabs(x)< x_region_){
        if(0 < y && y < y_region_){
          //std::cout << "x : " << x << ", y " << y << ", dist " << laser->ranges[i] << std::endl;
          //std::cout << "========== laser ranges index : " << i << " =========" << std::endl;
          return true;
        }
      } 
  }

  return false;
}

//TODO:
void PurePursuit::ComputeVelocities()
{
  // Get the current robot pose
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; idx_ < path_->poses.size(); idx_++)
    {
      if (Distance(path_->poses[idx_].pose.position, tf.transform.translation) > ld_)
      {

        // Transformed lookahead to base_link frame is lateral error
        KDL::Frame F_bl_ld = TransformToBaseLink(path_->poses[idx_].pose, tf.transform);
        lookahead_.transform.translation.x = F_bl_ld.p.x();
        lookahead_.transform.translation.y = F_bl_ld.p.y();
        lookahead_.transform.translation.z = F_bl_ld.p.z();
        F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);

        //std::cout << "=======" << idx_ << "========" << std::endl;

        break;
      }
    }

    //path路径不能为空，同时idex要大于等于path_.pose.size()
    if (!path_->poses.empty() && idx_ >= path_->poses.size())
    {
      // We are approaching the goal,
      // This is the pose of the goal w.r.t. the base_link frame
      KDL::Frame F_bl_end = TransformToBaseLink(path_->poses.back().pose, tf.transform);

      if (fabs(F_bl_end.p.x()) <= pos_tol_)
      {
        // We have reached the goal
        goal_reached_ = true;

        // Reset the path
        ResetPath();
        ResetID();
      }
      else
      {
        //参考plug_in_pure_pursuit：当global_plan距离大于look_ahead时，选择最后一个点
        lookahead_.transform.translation.x = F_bl_end .p.x();
        lookahead_.transform.translation.y = F_bl_end .p.y();
        lookahead_.transform.translation.z = F_bl_end .p.z();
        F_bl_end .M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);
      }
    }

    //TODO: 找到lookahead_，通过look_ahead进行速度规划，控制机器人前进
    if (!goal_reached_)
    {

      PublishVisual(lookahead_);             //可视化geometry_msgs::TransformStamped点，在机器人坐标系下
      //PublishVisual(path_.poses[idx_]);    //可视化path中的点，即map坐标系下的path.pose
      // We are tracking.
      //std::cout << "v_" << v_ << std::endl;
      //std::cout << "v_max_" << v_max_ << std::endl;

      if(v_ > v_max_){
        v_ = v_max_;
      }

      // Compute the angular velocity.
      // Lateral error is the y-value of the lookahead point (in base_link frame)
      double yt = lookahead_.transform.translation.y;
      double ld_2 = ld_ * ld_;

      //TAG:
      if(yt < 0){
        cmd_vel_.angular.z = std::max(2*v_ / ld_2 * yt, -w_max_);
        //std::cout << "w_max_ = " << -w_max_ << std::endl;
        //std::cout << "cmd_vel_.angular.z = " << std::max(2*v_ / ld_2 * yt, -w_max_) << std::endl;
      }
      else{
        cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );
        //std::cout << "w_max_ = " << w_max_ << std::endl;
        //std::cout << "cmd_vel_.angular.z = " << std::min( 2*v_ / ld_2 * yt, w_max_ ) << std::endl;
      }
      
      // Set linear velocity for tracking.
      cmd_vel_.linear.x = v_;
      //std::cout << "cmd_vel_.linear.x =" << v_ << std::endl;

      //TAG: 添加停障功能
      // if(check_front_obstacle_){
      //   cmd_vel_.angular.z = 0.0;
      //   cmd_vel_.linear.x = 0.0;
      // }
    }
    else
    {
      // We are at the goal!
      // Stop the vehicle
      
      // The lookahead target is at our current pose.
      lookahead_.transform = geometry_msgs::Transform();
      lookahead_.transform.rotation.w = 1.0;
      
      // Stop moving.
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
    }

    //std::cout << "cmd_vel_.angular.z = " << cmd_vel_.angular.z << std::endl;
    //std::cout << "cmd_vel_.linear.x = " << cmd_vel_.linear.x << std::endl;

    // Publish the lookahead target transform.
    lookahead_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(lookahead_);
    
    // Publish the velocities
    pub_vel_.publish(cmd_vel_);
  }
  catch (tf2::TransformException &ex)
  {
    std::cout << "====WARN!=====" << std::endl;
    ROS_WARN_STREAM(ex.what());
  }
}

//判断pure_pursuit中的path是否初始化完成
bool PurePursuit::PathInit()
{
  if (path_->poses.size() > 0)
  {
    goal_reached_ = false;
    return true;
  }
  else
  {
    goal_reached_ = true;
    return false;
  }
}

KDL::Frame PurePursuit::TransformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  return F_map_tf.Inverse()*F_map_pose;
}

void PurePursuit::Run()
{
  ros::Rate loop_rate(100);

  while(ros::ok()){
     ComputeVelocities();

     if(goal_reached_ || interrupt_request_){
       break;
     }

     loop_rate.sleep();
  }
}

// int main(int argc, char**argv)
// {
//   ros::init(argc, argv, "pure_pursuit");

//   vector<vector<double>> path_points{{2.86, -5.33}, {-0.83, -8.91}, {-5.04, -8.94}, {-6.66, -0.701}, {-4.61, 3.52}, {-0.054, 4.3}, {0.073, 2.82}, {-5.06, 1.43}, {-5.43, -5.62}, {1.4, -8.06}, {3.58, -3.54}};

//   PurePursuit controller;

//   controller.PathInit(path_points);

//   controller.Run();

//   return 0;
// }
