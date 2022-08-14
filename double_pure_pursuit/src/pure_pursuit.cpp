#include </home/zk/zk/pure_pursuit/cmd_pursuit_ws/src/double_pure_pursuit/include/pure_pursuit.hpp>
//#include "pure_pursuit.hpp"

PurePursuit::PurePursuit(std::vector<std::vector<double>> path_points) : ld_(1.0), v_max_(0.2), v_(v_max_), w_max_(1.0), pos_tol_(0.1), idx_(0),
                             goal_reached_(true), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("base_link"),
                             lookahead_frame_id_("lookahead"),pub_path_(path_points)
{
  // Get parameters from the parameter server
  nh_private_.param<double>("wheelbase", L_, 1.0);                          //轴距
  nh_private_.param<double>("lookahead_distance", ld_, 0.6);                //前沿点距离
  nh_private_.param<double>("linear_velocity", v_, 0.1);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 1.0);        //最大角速度
  nh_private_.param<double>("position_tolerance", pos_tol_, 0.1);           //位置误差
  nh_private_.param<double>("steering_angle_velocity", delta_vel_, 100.0);  //转角速度
  nh_private_.param<double>("acceleration", acc_, 100.0);                   //加速度
  nh_private_.param<double>("jerk", jerk_, 100.0);
  nh_private_.param<double>("steering_angle_limit", delta_max_, 1.57);      //转角最大
  nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "base_link");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");
  // Frame attached to midpoint of front axle (for front-steered vehicles).
  nh_private_.param<string>("ackermann_frame_id", acker_frame_id_, "rear_axle_midpoint");

  // Populate messages with static data
  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;

  cmd_acker_.header.frame_id = acker_frame_id_;
  cmd_acker_.drive.steering_angle_velocity = delta_vel_;
  cmd_acker_.drive.acceleration = acc_;
  cmd_acker_.drive.jerk = jerk_;
  
  //sub_path_ = nh_.subscribe("/trajectory", 1, &PurePursuit::receivePath, this);
  sub_odom_ = nh_.subscribe("/odom", 1, &PurePursuit::ComputeVelocities, this);
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_acker_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_acker", 1);
  setpoint_pub_ = nh_.advertise<visualization_msgs::Marker>("/set_point", 1);             //mark point pub

  std::cout << "START11111: v_" << v_ << std::endl;
  std::cout << "START11111: v_max_" << v_max_ << std::endl;
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
          p.pose.orientation.w =  current_pose.pose.orientation.w;
          p.pose.orientation.x = current_pose.pose.orientation.x;
          p.pose.orientation.y = current_pose.pose.orientation.y;
          p.pose.orientation.z = current_pose.pose.orientation.z;

          p.scale.x = p.scale.y = p.scale.z = 0.2;

          p.color.a = p.color.r = 1.0;
          p.color.g = p.color.b = 0.0;

         setpoint_pub_.publish(p);
}

//TODO:
void PurePursuit::ComputeVelocities(const nav_msgs::OdometryConstPtr& odom)
{
  // The velocity commands are computed, each time a new Odometry message is received.
  // Odometry is not used directly, but through the tf tree.

  // Get the current robot pose
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; idx_ < path_.poses.size(); idx_++)
    {
      if (Distance(path_.poses[idx_].pose.position, tf.transform.translation) > ld_)
      {

        // Transformed lookahead to base_link frame is lateral error
        KDL::Frame F_bl_ld = TransformToBaseLink(path_.poses[idx_].pose, tf.transform);
        lookahead_.transform.translation.x = F_bl_ld.p.x();
        lookahead_.transform.translation.y = F_bl_ld.p.y();
        lookahead_.transform.translation.z = F_bl_ld.p.z();
        F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);

        std::cout << "=======" << idx_ << "========" << std::endl;
        
        // TODO: See how the above conversion can be done more elegantly
        // using tf2_kdl and tf2_geometry_msgs

        break;
      }
    }
    //退出循环后，Index的值为path_.pose.size(),即没有查询到distance > ld_，此时为靠近goal的状态

    //path路径不能为空，同时idex要大于等于path_.pose.size()
    if (!path_.poses.empty() && idx_ >= path_.poses.size())
    {
      //TODO:
      idx_ = idx_ - 1;
      // We are approaching the goal,
      // which is closer than ld

      // This is the pose of the goal w.r.t. the base_link frame
      KDL::Frame F_bl_end = TransformToBaseLink(path_.poses.back().pose, tf.transform);

      if (fabs(F_bl_end.p.x()) <= pos_tol_)
      {
        // We have reached the goal
        goal_reached_ = true;

        // Reset the path
        path_ = nav_msgs::Path();
      }
      else
      {
        //FIXME:
        //case 1:
        //参考plug_in_pure_pursuit：当global_plan距离大于look_ahead时，选择最后一个点
        lookahead_.transform.translation.x = F_bl_end .p.x();
        lookahead_.transform.translation.y = F_bl_end .p.y();
        lookahead_.transform.translation.z = F_bl_end .p.z();
        F_bl_end .M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);
          
        // case 2: 延长：
        // double roll, pitch, yaw;
        // F_bl_end.M.GetRPY(roll, pitch, yaw);
        // double k_end = tan(yaw); // Slope of line defined by the last path pose
        // double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
        // double a = 1 + k_end * k_end;
        // double b = 2 * l_end;
        // double c = l_end * l_end - ld_ * ld_;
        // double D = sqrt(b*b - 4*a*c);
        // double x_ld = (-b + copysign(D,v_)) / (2*a);
        // double y_ld = k_end * x_ld + l_end;
        
        // lookahead_.transform.translation.x = x_ld;
        // lookahead_.transform.translation.y = y_ld;
        // lookahead_.transform.translation.z = F_bl_end.p.z();
        // F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x,
        //                          lookahead_.transform.rotation.y,
        //                          lookahead_.transform.rotation.z,
        //                          lookahead_.transform.rotation.w);
      }
    }

    //TODO: 找到lookahead_，通过look_ahead进行速度规划，控制机器人前进
    if (!goal_reached_)
    {

      //publish_visual(lookahead_);             //可视化geometry_msgs::TransformStamped点，在机器人坐标系下
      PublishVisual(path_.poses[idx_]);    //可视化path中的点，即map坐标系下的path.pose
      // We are tracking.
      std::cout << "v_" << v_ << std::endl;
      std::cout << "v_max_" << v_max_ << std::endl;

      // Compute linear velocity.
      // Right now,this is not very smart :)
      v_ = copysign(v_max_, v_);//以v_的符号，返回v_max

      // Compute the angular velocity.
      // Lateral error is the y-value of the lookahead point (in base_link frame)
      double yt = lookahead_.transform.translation.y;
      double ld_2 = ld_ * ld_;

      //TAG:
      cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );

      // Compute desired Ackermann steering angle
      cmd_acker_.drive.steering_angle = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );
      
      // Set linear velocity for tracking.
      cmd_vel_.linear.x = v_;
      std::cout << "cmd_vel_.linear.x =" << v_ << std::endl;
      cmd_acker_.drive.speed = v_;

      cmd_acker_.header.stamp = ros::Time::now();
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

      cmd_acker_.header.stamp = ros::Time::now();
      cmd_acker_.drive.steering_angle = 0.0;
      cmd_acker_.drive.speed = 0.0;
    }

    // Publish the lookahead target transform.
    lookahead_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(lookahead_);
    
    // Publish the velocities
    pub_vel_.publish(cmd_vel_);
    
    // Publish ackerman steering setpoints
    pub_acker_.publish(cmd_acker_);
  }
  catch (tf2::TransformException &ex)
  {
    std::cout << "====WARN!=====" << std::endl;
    ROS_WARN_STREAM(ex.what());
  }
}

// void PurePursuit::ReceivePath(nav_msgs::Path& new_path)
// {
  
//   if (new_path.header.frame_id == map_frame_id_)
//   {
//     path_ = new_path;
//     //idx_ = 0;
//     if (new_path.poses.size() > 0)
//     {
//       goal_reached_ = false;
//       //std::cout << "sub_path_.shutdown()" << std::endl;
//       //sub_path_.shutdown();                                                       //当接收到path时，停止接收path
//     }
//     else
//     {
//       goal_reached_ = true;
//       ROS_WARN_STREAM("Received empty path!");
//     }
//   }
//   else
//   {
//     ROS_WARN_STREAM("The path must be published in the " << map_frame_id_
//                     << " frame! Ignoring path in " << new_path.header.frame_id
//                     << " frame!");
//   }
// }

void PurePursuit::PathInit()
{
  pub_path_.Run();    //执行MakeLine，生成直线
  path_ = pub_path_.path_;  //赋值给当前path
  if (path_.poses.size() > 0)
  {
    goal_reached_ = false;
  }
  else
  {
    goal_reached_ = true;
    ROS_WARN_STREAM("Received empty path!");
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
  pub_path_.LineVisual();   //发布path
  ros::spinOnce();
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "pure_pursuit");

  vector<vector<double>> path_points{{3.21, -1.46}, {3.19, -5.54}, {-0.85, -8.52}, {-4.37, -5.28}};
  
  PurePursuit controller(path_points);

  controller.PathInit();

  ros::Rate loop_rate(100);   //频率更改
  while(ros::ok())
    {
      controller.Run();
    }

  return 0;
}
