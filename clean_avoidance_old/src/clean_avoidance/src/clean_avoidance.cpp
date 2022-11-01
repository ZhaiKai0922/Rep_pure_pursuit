#include <vector>
#include <string>

#include <ros/ros.h>

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

using namespace std;

class CleanAvoidance{
    public:
    CleanAvoidance(tf2_ros::Buffer& tf);
    ~CleanAvoidance();
    void ExecuteCb(const clean_avoidance::CleanAvoidanceGoalConstPtr& clean_goal);  //action回调函数，接收目标点
    void MakeLine(const vector<double>& first, const vector<double>& second);    //两点生成直线
    void PathInit(const vector<vector<double>>& goals);
    void FiltePath();     //根据costmap实时修改直线
    void LineVisual();

    tf2_ros::Buffer& tf_;
    ros::Publisher vel_pub_;
    ros::Publisher path_pub_;

    actionlib::SimpleActionServer<clean_avoidance::CleanAvoidanceAction>* as_;

    costmap_2d::Costmap2DROS* planner_costmap_ros_;
    costmap_2d::Costmap2DROS* controller_costmap_ros_;

    private:
    nav_msgs::Path path_;
    double path_resolution_ = 0.1;
    double safe_distance_ = 0.5;

    vector<double> robot_pose_{0.0, 0.0};

};

CleanAvoidance::CleanAvoidance(tf2_ros::Buffer& tf):
    tf_(tf), 
    as_(NULL), 
    planner_costmap_ros_(NULL),
    controller_costmap_ros_(NULL){
        
    //创建action server，接收外部的目标请求，生成完整的路线
    //actionlib会启动一个线程，当外部请求到来时，调用CleanAvoidance::ExecuteCb回调函数处理
    as_ = new actionlib::SimpleActionServer<clean_avoidance::CleanAvoidanceAction>(ros::NodeHandle(), "clean_avoidance", boost::bind(&CleanAvoidance::ExecuteCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("trajectory", 1, true);

    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //启动执行action处理外部请求
    as_->start();
}

CleanAvoidance::~CleanAvoidance(){
    if(as_ != NULL){
        delete as_;
    }
    if(planner_costmap_ros_ != NULL){
        delete planner_costmap_ros_;
    }
    if(controller_costmap_ros_ != NULL){
        delete controller_costmap_ros_;
    }
}

void CleanAvoidance::ExecuteCb(const clean_avoidance::CleanAvoidanceGoalConstPtr& clean_goal){
    vector<vector<double>> temp_points;
    vector<double> temp_goal;

    temp_goal.push_back(clean_goal->goal_x);
    temp_goal.push_back(clean_goal->goal_y);
    
    temp_points.push_back(temp_goal);

    //传入目标点 制作直线 
    PathInit(temp_points);

    cout << "Cycle ...................... " << endl;

    ros::Rate loop_rate(50);
    while(ros::ok()){

        //循环遍历直线的点 查找对应cost值 修改直线
        FiltePath();

        //发布直线
        LineVisual();

        loop_rate.sleep();
    }

    //直至机器人到达目标位置 退出循环
    //as_->setSucceeded(clean_avoidance::CleanAvoidanceResult(), "Goal reached.");
}

//输入起始点与终止点坐标：(x1, y1)，(x2, y2)，输出直线
void CleanAvoidance::MakeLine(const vector<double>& first, const vector<double>& second){
    double x_first = first[0];
    double y_first = first[1];

    double x_second = second[0];
    double y_second = second[1];

    ros::Time current_time;
    geometry_msgs::PoseStamped this_pose_stamped;
    current_time = ros::Time::now();

    double dist = hypot(x_second - x_first, y_second - y_first);    //计算斜边长度
    double ang  = atan2(y_second - y_first, x_second - x_first);   //返回单位值为弧度，取值范围为(-pi, pi)

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(ang);   //通过倾斜角获得quaternion

    this_pose_stamped.pose.position.x = x_first;
    this_pose_stamped.pose.position.y = y_first;
    this_pose_stamped.pose.position.z = 0;
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = "map";
    path_.poses.push_back(this_pose_stamped);

    double npoints = dist/path_resolution_;

    for(int j = 0; j < npoints; j++){
        current_time = ros::Time::now();
        x_first = x_first + path_resolution_ * cos(ang);
        y_first = y_first + path_resolution_ * sin(ang);

        this_pose_stamped.pose.position.x = x_first;
        this_pose_stamped.pose.position.y = y_first;
        this_pose_stamped.pose.position.z = 0;

        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "map";

        path_.poses.push_back(this_pose_stamped);

    }
    cout << path_.poses.size() << endl;
}

//传入多个目标点
void CleanAvoidance::PathInit(const vector<vector<double>>& goals){
    //tf::StampedTransform baselink_to_map;
    geometry_msgs::TransformStamped baselink_to_map;
    try{
        baselink_to_map = tf_.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch(tf2::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    //robot_pose_[0] = baselink_to_map.getOrigin().x();
    //robot_pose_[1] = baselink_to_map.getOrigin().y();
    robot_pose_[0] = baselink_to_map.transform.translation.x;
    robot_pose_[1] = baselink_to_map.transform.translation.y;

    int size = goals.size();

    if(size > 0){
        //case1: base_link作为起点
        for(int i = 0; i < size; i++){
            if(i == 0){
                MakeLine(robot_pose_, goals[i]);
            }
            else{
                MakeLine(goals[i-1], goals[i]);
            }
        }
    }

    // case2: 直接读取points
    // if(size > 1){
    //     for(int i = 1; i < size; i++){
    //         MakeLine(goals[i-1], goals[i]);
    //     }
    // }

    else{
        ROS_ERROR("SIZE OF GOALS IS ZERO!");
    }
   
   //FIXME:
    cout << "planner_costmap_ros_ ->getCostmap()->getResolution()" << 
    planner_costmap_ros_ ->getCostmap()->getResolution() << endl;

}

//path可视化
void CleanAvoidance::LineVisual(){
    ros::Time current_time = ros::Time::now();
    path_.header.stamp = current_time;
    path_.header.frame_id = "map";
    path_pub_.publish(path_);
}

//读取cost值，根据cost值修改路径
void CleanAvoidance::FiltePath(){
    if(path_.poses.empty()){
        ROS_INFO("CleanAvoidance::FiltePath: plan is EMPTY!");
        return;
    }

    costmap_2d::Costmap2D* my_costmap = planner_costmap_ros_ ->getCostmap();

    int safe_cell = (int)( safe_distance_/my_costmap->getResolution() );

    if(safe_cell < 1){
        ROS_INFO("The safety distance is too small !");
        return;
    }

    int path_size = path_.poses.size();

    geometry_msgs::PoseStamped temp_point;              //当前点
    geometry_msgs::PoseStamped before_point;           //前一个点
    geometry_msgs::PoseStamped next_point;                //下一个点
    geometry_msgs::PoseStamped nearest_obstacle;   //最近障碍物的点

    unsigned int x_min;   //保持安全距离，cell的x最小值
    unsigned int x_max;   //保持安全距离，cell的x最大值
    unsigned int y_min;   //保持安全距离，cell的y最小值
    unsigned int y_max;   //保持安全距离，cell的y最大值
    unsigned int cell_x;    //单位值m，转换到地图下的像素坐标cell_x
    unsigned int cell_y;    //单位值m，转换到地图下的像素坐标cell_y
    
    //循环变量path中的全部点，查找当前point在安全距离范围内是否存在占据状态的cell
    for(int i = 0; i < path_size; i++){
        temp_point = path_.poses[i];
        before_point = (i > 0) ? path_.poses[i-1] : path_.poses[i];
        next_point = (i < path_size-1) ? path_.poses[i+1] : path_.poses[i];

        my_costmap -> worldToMap(temp_point.pose.position.x, temp_point.pose.position.y, cell_x, cell_y);
        x_min = (cell_x > safe_cell) ? cell_x - safe_cell : cell_x;
        x_max = (cell_x + safe_cell < my_costmap->getSizeInCellsX()) ? cell_x + safe_cell : cell_x;
        y_min = (cell_y > safe_cell) ? cell_y - safe_cell : cell_y;
        y_max = (cell_y +safe_cell < my_costmap->getSizeInCellsY()) ? cell_y + safe_cell : cell_y;

        vector<geometry_msgs::Point> obstacle_vec;  //检测安全距离范围内的障碍物，将障碍物存放到obstacle_vec中
        geometry_msgs::Point obstacle;
        obstacle_vec.clear();

        //在安全距离范围内查找cell的cost值
        for(unsigned int j = x_min; j < x_max; j++){
            for(unsigned int k = y_min; k < y_max; k++){
                //......
                if(my_costmap->getCost(j, k) != costmap_2d::FREE_SPACE){
                    //将障碍物的cell值，转变为单位值为m
                    my_costmap->mapToWorld(j, k, obstacle.x, obstacle.y);

                    //将障碍物的实际距离存储到obstacle中
                    obstacle_vec.push_back(obstacle);
                }
            }
        }

        //检查障碍物是否存在于同一侧
        if(obstacle_vec.empty() != true){

            // bool same_side_flag = false;
            // if(next_point.pose.position.x != before_point.pose.position.x){
            //     double lk = 0;
            //     double lb = 0;
            //     double ly = 0;
            //     double num = 0;

            //     lk = (next_point.pose.position.y - before_point.pose.position.y) / (next_point.pose.position.x - before_point.pose.position.x);
            //     lb = next_point.pose.position.y - lk * next_point.pose.position.x;

            //     for(int m = 0; m < obstacle_vec.size(); m++){
            //         ly = lk * obstacle_vec[m].x + lb;
            //         if(ly != 0){
            //             break;
            //         }
            //     }

            //     //FIXME:
            //     for(int m = 0; m < obstacle_vec.size(); m++){
            //         num = ly * (lk * obstacle_vec[m].x + lb);
            //         if(num < 0){
            //             //此时obstacle不在同一侧
            //             same_side_flag = true;
            //             break;
            //         }
            //     }   
            // }else{
            //     double const_x = next_point.pose.position.x;
            //     double err = 0;
            //     double num = 0;
            //     for(int m = 0; m < obstacle_vec.size(); m++){
            //         err = const_x - obstacle_vec[m].x;
            //         if(err != 0){
            //             break;
            //         }
            //     }
            //     for(int m = 0; m < obstacle_vec.size(); m++){
            //         num = err * (const_x - obstacle_vec[m].x);
            //         if(num < 0){
            //             //此时obstacle不在同一侧
            //             same_side_flag = true;
            //             break;
            //         }
            //     }
            // }

            // if(same_side_flag == true){
            //     ROS_INFO("These points are not on the same side!");
            //     continue;
            // }

            double distance = 0;
            double min_distance_obst = 10000.0;
            int min_obst_index = 0;
            double diff_x, diff_y;
            for(int m = 0; m < obstacle_vec.size(); m++){
                diff_x = obstacle_vec[m].x - temp_point.pose.position.x;
                diff_y = obstacle_vec[m].y - temp_point.pose.position.y;
                distance = sqrt(diff_x * diff_x + diff_y * diff_y);
                if(distance < min_distance_obst){
                    min_distance_obst = distance;
                    min_obst_index = m;
                }
            }

            if(safe_distance_ - min_distance_obst < 0.0){
                continue;
            }

            //最近障碍物目标位置
            nearest_obstacle.pose.position.x = obstacle_vec[min_obst_index].x;
            nearest_obstacle.pose.position.y = obstacle_vec[min_obst_index].y;

            distance = safe_distance_ - min_distance_obst;
            
            double err_x;
            double err_y;
            double theta;
            double finally_x;
            double finally_y;
            theta = atan2(temp_point.pose.position.y - nearest_obstacle.pose.position.y, temp_point.pose.position.x - nearest_obstacle.pose.position.x);

            err_x = distance * cos(theta);
            err_y = distance * sin(theta);
            finally_x = temp_point.pose.position.x +err_x;
            finally_y = temp_point.pose.position.y + err_y;

            my_costmap->worldToMap(finally_x, finally_y, cell_x, cell_y);

            if(my_costmap->getCost(cell_x, cell_y) == costmap_2d::FREE_SPACE){
                path_.poses[i].pose.position.x = finally_x;
                path_.poses[i].pose.position.y = finally_y;
            }

        }
        
    }

}


int main(int argc, char** argv){
    ros::init(argc, argv, "clean_avoidance_node");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    CleanAvoidance clean_avoidance(buffer);

    ros::spin();

    return 0;
}