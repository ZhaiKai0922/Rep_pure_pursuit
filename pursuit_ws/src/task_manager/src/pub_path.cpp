
//#include </home/zk/zk/pure_pursuit/cmd_pursuit_ws/src/double_pure_pursuit/include/pub_path.hpp>
#include "pub_path.hpp"
using namespace std;

PubPath::PubPath():pathResolution_(0.1), nh_("~"){
    path_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory",1, true);
    ros::Time current_time = ros::Time::now();
    path_.header.stamp = current_time;
    path_.header.frame_id = "map";
}

void PubPath::ReceivePoints(const vector<vector<double>>& points){
    points_ = points;
}

void PubPath::MakeLine(const vector<double>& first, const vector<double>& second){
    double x_fir = first[0];
    double y_fir = first[1];

    double x_sec = second[0];
    double y_sec = second[1];

    ros::Time current_time;
    geometry_msgs::PoseStamped this_pose_stamped;
    current_time = ros::Time::now();
    
    //path_.header.stamp = current_time;
    //path_.header.frame_id = "map";

    double dist = hypot(x_sec - x_fir, y_sec - y_fir);  //计算斜边sqrt(x*x + y*y)
    double ang = atan2(y_sec - y_fir, x_sec - x_fir);  //返回单位值为弧度，取值范围为(-pi, pi)

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(ang);

    this_pose_stamped.pose.position.x = x_fir;
    this_pose_stamped.pose.position.y = y_fir;
    this_pose_stamped.pose.position.z = 0;
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path_.poses.push_back(this_pose_stamped);

    double npoints = dist / pathResolution_;

    for(int j = 0; j < npoints; j++){
        current_time = ros::Time::now();
        x_fir = x_fir + pathResolution_ * cos(ang);
        y_fir = y_fir + pathResolution_ * sin(ang);

        this_pose_stamped.pose.position.x = x_fir;
        this_pose_stamped.pose.position.y = y_fir;
        this_pose_stamped.pose.position.z = 0;

        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="map";
        path_.poses.push_back(this_pose_stamped);
    }

    // current_time = ros::Time::now();
    // this_pose_stamped.pose.position.x = x_sec;
    // this_pose_stamped.pose.position.y = y_sec;
    // this_pose_stamped.pose.position.z = 0;
    // this_pose_stamped.pose.orientation.x = goal_quat.x;
    // this_pose_stamped.pose.orientation.y = goal_quat.y;
    // this_pose_stamped.pose.orientation.z = goal_quat.z;
    // this_pose_stamped.pose.orientation.w = goal_quat.w;
    // this_pose_stamped.header.stamp=current_time;
    // this_pose_stamped.header.frame_id="map";
    // path_.poses.push_back(this_pose_stamped);

    std::cout << path_.poses.size() << std::endl;
}

void PubPath::LineVisual(){
    path_pub_.publish(path_);
    std::cout <<"============================================!!!!!" << std::endl;
}

void PubPath::ResetPath()
{
    //path_ = nav_msgs::Path();
    path_.poses.clear();
}

void PubPath::Run(const vector<vector<double>>& points){
    ReceivePoints(points);

    tf::StampedTransform transform;
    try{
        listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    robot_point_[0] = transform.getOrigin().x();
    robot_point_[1] = transform.getOrigin().y();

    int size = points_.size();
    
    if(size > 1){
        //Case1:  robot base_link开始
        // for(int i = 0; i < size; i ++){
        //     if(i == 0){
        //         cout << "robot_point_:" << "[" << robot_point_[0] << ", " << robot_point_[1] << "]"
        //                   <<" ->->->" <<  "points_" << i << "[" << points_[i][0]<< ", " << points_[i][1] << endl;
        //         MakeLine(robot_point_, points_[i]);
        //     }
        //     else{
        //          cout << "points_:" << i-1<<"[" << points_[i-1][0] << ", " << points_[i-1][1] << "]"
        //                   <<" ->->->" <<  "points_" << i << "[" << points_[i][0]<< ", " << points_[i][1] << endl;
        //         MakeLine(points_[i - 1],points_[i]);
        //     }
        // }
    
        //Case 2: 直接读取points
        for(int i = 1; i < size; i ++){
            cout << "points_:" << i-1<<"[" << points_[i-1][0] << ", " << points_[i-1][1] << "]"
                        <<" ->->->" <<  "points_" << i << "[" << points_[i][0]<< ", " << points_[i][1] << endl;
            MakeLine(points_[i - 1],points_[i]);
        }
    }
    else if(size = 1){
        ROS_ERROR("Size of Points is ONE!");
    }
    else{
        ROS_ERROR("Size of Points is ZERO!");
    }

}