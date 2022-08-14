
#include </home/zk/zk/pure_pursuit/cmd_pursuit_ws/src/double_pure_pursuit/include/pub_path.hpp>
using namespace std;

PubPath::PubPath(vector<vector<double>> points):points_(points),pathResolution_(0.2), nh_("~"){
    path_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory",1, true);
    //odom_sub_ = nh_.subscribe("/odom", 10, &lin_pub::odomCallback, this);
    ros::Time current_time = ros::Time::now();
    path_.header.stamp = current_time;
    path_.header.frame_id = "map";
}

void PubPath::MakeLine(vector<double> first, vector<double> second){
    double x_fir = first[0];
    double y_fir = first[1];

    double x_sec = second[0];
    double y_sec = second[1];

    ros::Time current_time;
    geometry_msgs::PoseStamped this_pose_stamped;
    current_time = ros::Time::now();
    
    //path_.header.stamp = current_time;
    //path_.header.frame_id = "map";

    double dist = hypot(x_sec - x_fir, y_sec - y_fir);
    double ang = atan2(y_sec - y_fir, x_sec - x_fir);

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

    current_time = ros::Time::now();
    this_pose_stamped.pose.position.x = x_sec;
    this_pose_stamped.pose.position.y = y_sec;
    this_pose_stamped.pose.position.z = 0;
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path_.poses.push_back(this_pose_stamped);

    std::cout << path_.poses.size() << std::endl;
}

void PubPath::LineVisual(){
    path_pub_.publish(path_);
}

void PubPath::Run(){
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

    //Case1:  robot base_link开始
    // for(int i = 0; i < size; i ++){
    //     if(i == 0){
    //         cout << "robot_point_:" << "[" << robot_point_[0] << ", " << robot_point_[1] << "]"
    //                   <<" ->->->" <<  "points_" << i << "[" << points_[i][0]<< ", " << points_[i][1] << endl;
    //         make_line(robot_point_, points_[i]);
    //     }
    //     else{
    //          cout << "points_:" << i-1<<"[" << points_[i-1][0] << ", " << points_[i-1][1] << "]"
    //                   <<" ->->->" <<  "points_" << i << "[" << points_[i][0]<< ", " << points_[i][1] << endl;
    //         make_line(points_[i - 1],points_[i]);
    //     }
    // }
    
    //Case 2: 直接读取points
    for(int i = 1; i < size; i ++){
        cout << "points_:" << i-1<<"[" << points_[i-1][0] << ", " << points_[i-1][1] << "]"
                      <<" ->->->" <<  "points_" << i << "[" << points_[i][0]<< ", " << points_[i][1] << endl;
        MakeLine(points_[i - 1],points_[i]);
    }

    // ros::Rate loop_rate(10);
    // while(ros::ok()){
    //     LineVisual();
    //     loop_rate.sleep();
    // }
}

// int main(int argc, char** argv){
//     ros::init (argc, argv, "line_pub");
    
//     vector<vector<double>> points{{3.21, -1.46}, {3.19, -5.54}, {-0.85, -8.52}, {-4.37, -5.28}};
//     PubPath lin_pub_exam(points);

//     //ros::spinOnce();
//     cout << "run()" << endl;
//     lin_pub_exam.run();
    
//     return 0;
// }