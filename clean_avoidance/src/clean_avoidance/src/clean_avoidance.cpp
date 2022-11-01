#include "clean_avoidance/clean_avoidance.hpp"

using namespace std;

CleanAvoidance::CleanAvoidance(tf2_ros::Buffer& tf):
    tf_(tf), 
    as_(NULL), 
    planner_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"){
    
    std::string global_planner;
    //创建action server，接收外部的目标请求，生成完整的路线
    //actionlib会启动一个线程，当外部请求到来时，调用CleanAvoidance::ExecuteCb回调函数处理
    as_ = new actionlib::SimpleActionServer<clean_avoidance::CleanAvoidanceAction>(ros::NodeHandle(), "clean_avoidance", boost::bind(&CleanAvoidance::ExecuteCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("first_search_waypoints_delta", first_search_waypoints_delta_, 30);
    private_nh.param("search_waypoints_delta", search_waypoints_delta_, 15);
    private_nh.param("search_waypoints_size", search_waypoints_size_, 76);
    private_nh.param("reduce_start_points", reduce_start_points_, 5);
    private_nh.param("number_greater_index", number_greater_index_, 10);
    private_nh.param("number_less_index", number_less_index_, 10);
    private_nh.param("cycle_reduce_start_points", cycle_reduce_start_points_, 3);

    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("trajectory", 1, true);

    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    try{
        planner_ = bgp_loader_.createInstance(global_planner);
        planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    }catch(const pluginlib::PluginlibException& ex){
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
        exit(1);
    }

    planner_costmap_ros_->start();

    my_costmap_ = planner_costmap_ros_ ->getCostmap();

    pure_pursuit_ = boost::make_shared<PurePursuit>();
    //FIXME:
    pub_vel_thread_ = boost::make_shared<boost::thread>(boost::bind(&CleanAvoidance::PubVelThread, this));

    //启动执行action处理外部请求
    as_->start();

    cout << "first_search_waypoints_delta : " << first_search_waypoints_delta_ << endl;
    cout << "search_waypoints_delta : " << search_waypoints_delta_ << endl;
    cout << "search_waypoints_size : " << search_waypoints_size_ << endl;
    cout << "reduce_start_points : " << reduce_start_points_ << endl;
    cout << "number_greater_index : " << number_greater_index_ << endl;
    cout << "number_less_index : " << number_less_index_ << endl;
    cout << "cycle_reduce_start_points : " << cycle_reduce_start_points_ << endl;
}

CleanAvoidance::~CleanAvoidance(){
    if(as_ != NULL){
        delete as_;
    }
    if(planner_costmap_ros_ != NULL){
        delete planner_costmap_ros_;
    }

    planner_.reset();
}

//纯路径跟踪线程
void CleanAvoidance::PubVelThread(){
    ros::Rate loop_rate(40);

    while(ros::ok()){
        if(path_init_flag_ && pub_vel_thread_flag_){
            cout << "pure_pursuit_->Run()" << endl;
            pure_pursuit_->Run();
            if(pure_pursuit_->goal_reached_){
                pub_vel_thread_flag_ = false;
                ResetPath();
                PubZeroVel();
            }
            else if(pure_pursuit_->interrupt_request_){
                cout << "pure_pursuit->interrupt_request = true" << endl;
                pub_vel_thread_flag_ = false;
                pure_pursuit_->interrupt_request_ = false;
                PubZeroVel();
            }
        }

        loop_rate.sleep();
    }
}

void CleanAvoidance::PubZeroVel(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    for(int i = 0; i < 8; i++){
        vel_pub_.publish(cmd_vel);
    }
}

bool CleanAvoidance::Init(vector<vector<double>> points){
    if(PathInit(points)){
        if(!FixPath()){
            cout << "Init : Aborting because FIX PATH FAILED !" << endl;
            ResetPath();
            return false;
        }else{
            stop_flag = false;
            cancel_flag = false;
            return true;
        }
    }else{
        return false;
    }
}

float CleanAvoidance::ReadTaskPercentage(){
    return pure_pursuit_->ReadTaskPercentage();
}

bool CleanAvoidance::Run(){
    pub_vel_thread_flag_ = true;
    cout << "Into the loop ..." << endl;

    ros::Rate loop_rate(40);

    while(ros::ok()){
        ros::spinOnce();
        
        if(!stop_flag){
            if(pure_pursuit_->goal_reached_){
                usleep(30000);
                LineVisual();
                cout << "CleanAvoidance: Goal Reached !" << endl;
                return true;
            }

            if(cancel_flag){
                ResetPath();
                return true;
            }

            if(!CycleFixPath()){
                cout << "FIX PATH FAILED !" << endl;
            }

            LineVisual();
        }

        loop_rate.sleep();
    }

}

bool CleanAvoidance::Start(){
    pub_vel_thread_flag_ = true;
    stop_flag = false;
    usleep(30000);
    return true;
}

bool CleanAvoidance::Stop(){
    if(path_init_flag_ && pub_vel_thread_flag_){
        pure_pursuit_->interrupt_request_ = true;
        stop_flag = true;
        usleep(30000);
        return true;
    }else{
        stop_flag = true;
        usleep(30000);
        return true;
    }
}

bool CleanAvoidance::Cancel(){
    if(path_init_flag_ && pub_vel_thread_flag_){
        pure_pursuit_->interrupt_request_ = true;
        cancel_flag = true;
        usleep(30000);
        return true;
    }else{
        cancel_flag = true;
        usleep(30000);
        return true;
    }
}

void CleanAvoidance::ExecuteCb(const clean_avoidance::CleanAvoidanceGoalConstPtr& clean_goal){
    vector<vector<double>> temp_points;
    vector<double> temp_goal;

    temp_goal.push_back(clean_goal->goal_x);
    temp_goal.push_back(clean_goal->goal_y);
    
    temp_points.push_back(temp_goal);

    //传入目标点 制作直线 
    if(PathInit(temp_points)){
        if(!FixPath()){
            as_->setAborted(clean_avoidance::CleanAvoidanceResult(), "INIT: Aborting because FIX PATH FAILED !");
            cout << "INIT: Aborting because FIX PATH FAILED !" << endl;
            ResetPath();
            return;
        }
        pub_vel_thread_flag_ = true;

        cout << "Cycle ...................... " << endl;

        ros::Rate loop_rate(40);
        while(ros::ok()){
            if(pure_pursuit_->goal_reached_){
                usleep(30000);
                LineVisual();
                //直至机器人到达目标位置 退出循环
                as_->setSucceeded(clean_avoidance::CleanAvoidanceResult(), "Goal reached.");
                cout << "CleanAvoidance: Goal Reached !" << endl;
                return;
            }
            //FIXME:
            if(as_->isPreemptRequested()){
                if(as_->isNewGoalAvailable()){
                    pure_pursuit_->interrupt_request_ = true;
                    usleep(30000);
                    ResetPath();
                    clean_avoidance::CleanAvoidanceGoal new_goal = *as_->acceptNewGoal();
                    temp_goal.clear();
                    temp_points.clear();
                
                    temp_goal.push_back(new_goal.goal_x);
                    temp_goal.push_back(new_goal.goal_y);
                    temp_points.push_back(temp_goal);
                    
                    //传入目标点 制作直线 
                    PathInit(temp_points);
                    if(!FixPath()){
                        as_->setAborted(clean_avoidance::CleanAvoidanceResult(), "New Goal INIT: Aborting because FIX PATH FAILED !");
                        cout << "New Goal INIT: Aborting because FIX PATH FAILED !" << endl;
                        ResetPath();
                        return;
                    }
                    pub_vel_thread_flag_ = true;
                    cout << "New Goal Available .........." << endl;
                }
                else{
                    pure_pursuit_->interrupt_request_ = true;
                    ResetPath();
                    cout << "as->setPreempted(), break" << endl;
                    as_->setPreempted();
                    return;
                }
            }

            //循环遍历直线的点 查找对应cost值 修改直线
            if(!CycleFixPath()){
                cout << " FIX PATH FAILED !" << endl;
            }

            //发布直线
            LineVisual();

            loop_rate.sleep();
        }

    }

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
}

//Reset Path:
void CleanAvoidance::ResetPath(){
    pure_pursuit_->ResetPath();
    pure_pursuit_->ResetID();
    path_.poses.clear();
    path_init_flag_ = false;
}

//传入多个目标点
bool CleanAvoidance::PathInit(const vector<vector<double>>& goals){
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
        cout << "PathInit : SIZE OF GOALS IS ZERO!" << endl;
        return false;
    }

    //将生成的path传递给pure_pursuit_，实现路径跟踪
    pure_pursuit_->path_ = &path_;

    if(pure_pursuit_->PathInit()){
        cout << "PurePursuit: Path Init Succeed !" << endl;
        path_init_flag_ = true;
        return true;
    }else{
        cout << "PurePursuit: Failed to Init Path !" << endl;
        return false;
    }
}

//path可视化
void CleanAvoidance::LineVisual(){
    ros::Time current_time = ros::Time::now();
    path_.header.stamp = current_time;
    path_.header.frame_id = "map";
    path_pub_.publish(path_);
}

//初始化时，历遍路径点读取cost值，并根据cost值修改路径
bool CleanAvoidance::FixPath(){
    if(path_.poses.empty()){
        ROS_INFO("CleanAvoidance::FixPath: plan is EMPTY!");
        return true;
    }

    int safe_cell = (int)( safe_distance_/my_costmap_->getResolution() );

    if(safe_cell < 1){
        ROS_INFO("The safety distance is too small !");
        std::cout << "The safety distance is too small !" << std::endl;
        return false;
    }

    geometry_msgs::PoseStamped temp_point;              //当前点
    geometry_msgs::PoseStamped goal_point;           //goal点
    geometry_msgs::PoseStamped start_point;           //start点

    unsigned int x_min;   //保持安全距离，cell的x最小值
    unsigned int x_max;   //保持安全距离，cell的x最大值
    unsigned int y_min;   //保持安全距离，cell的y最小值
    unsigned int y_max;   //保持安全距离，cell的y最大值
    unsigned int cell_x;    //单位值m，转换到地图下的像素坐标cell_x
    unsigned int cell_y;    //单位值m，转换到地图下的像素坐标cell_y
    
    //循环变量path中的全部点，查找当前point在安全距离范围内是否存在占据状态的cell
    for(int index = 0; index < path_.poses.size(); index++){
        temp_point = path_.poses[index];

        my_costmap_->worldToMap(temp_point.pose.position.x, temp_point.pose.position.y, cell_x, cell_y);
        x_min = (cell_x > safe_cell) ? cell_x - safe_cell : cell_x;
        x_max = (cell_x + safe_cell < my_costmap_->getSizeInCellsX()) ? cell_x + safe_cell : cell_x;
        y_min = (cell_y > safe_cell) ? cell_y - safe_cell : cell_y;
        y_max = (cell_y +safe_cell < my_costmap_->getSizeInCellsY()) ? cell_y + safe_cell : cell_y;

        //在安全距离范围内查找cell的cost值
        for(unsigned int j = x_min; j < x_max; j++){
            for(unsigned int k = y_min; k < y_max; k++){
                    if(my_costmap_->getCost(j, k) == costmap_2d::LETHAL_OBSTACLE ||
                        my_costmap_->getCost(j, k) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                    avoid_flag_ = true;                                                                                  //出现cost值为占据，则表明该path点安全距离范围内存在障碍物，触发路径重新规划
                    int start_index = index;                                                                        //当前开始点start_index大小为index
                    int temp_index = -1;                                                                              //临时变量，接收FindMinIndex()返回的占据index
                    if(FindMinIndex(0, path_.poses.size(), temp_index)){       //查询一下当前轨迹最小占据index，赋值给start_index
                        if(temp_index < start_index){
                            start_index = temp_index;
                        }
                    }
                    start_waypoint_index_ = (start_index > reduce_start_points_) ? start_index - reduce_start_points_ : 0;
                    break;
                }
            }
            if(avoid_flag_ == true){
                cout << "Temp Point Index : " << index << endl;
                cout << "Start Waypoint Index : " << start_waypoint_index_ << endl;
                break;
            }
        }

        if(avoid_flag_){
            avoid_flag_ = false;
            bool found_path = false;
            int count = 0;                                                                              //目标点超过size的次数

            //该循环查找目标点，goal_waypoint_index
            for(int i = first_search_waypoints_delta_; i < search_waypoints_size_; i += search_waypoints_delta_){
                goal_waypoint_index_ = start_waypoint_index_ + i;
    
                if(goal_waypoint_index_ >= path_.poses.size()){
                    if(count > 0){
                        cout << "goal_waypoint_index >= path_.poses.size()" << endl;
                        break;
                    }
                    goal_waypoint_index_ = path_.poses.size() - 1;   //当第一次超过目标点时，将path最后一个点作为目标点
                    count++;
                }

                start_point = path_.poses[start_waypoint_index_];   //初始点
                goal_point = path_.poses[goal_waypoint_index_];    //目标点

                cout << "FixPath : planner_->makePlan ......" << endl;

                vector<geometry_msgs::PoseStamped> global_plan;
                found_path = planner_->makePlan(start_point, goal_point, global_plan);

                if(found_path && !global_plan.empty()){
                    end_of_avoid_index = goal_waypoint_index_;
                    MergeAvoidWaypoints(global_plan, end_of_avoid_index);
                    if(path_.poses.size() > 0){
                        //FIXME:
                        index = goal_waypoint_index_;
                        //ROS_INFO("Found GOAL at index = %d", goal_waypoint_index_);
                        cout << "Goal Waypoint Index : " << goal_waypoint_index_ << endl;
                        fix_path_ = true;
                        break;
                    }else{
                        found_path = false;
                    }
                }
            }

            if(fix_path_ == true){
                fix_path_ = false;
                //ROS_INFO("Fix path is TRUE !!! ");
                std::cout << "Fix path is TRUE !!!" << std::endl;
            }else{
                //ROS_ERROR("Can't find goal ,Fix path is FLASE !!!");
                std::cout << "Can't find goal, Fix path is FALSE !!!" << std::endl;
                return false;                         //当规划路径失败时，返回false
            }

        }
        
    }

    return true;
}

//循环过程中的路径修改，仅在触发开始后
bool CleanAvoidance::CycleFixPath(){
    if(path_.poses.empty()){
        ROS_INFO("CleanAvoidance::FixPath: plan is EMPTY!");
        return true;
    }

    int safe_cell = (int)( safe_distance_/my_costmap_->getResolution() );

    if(safe_cell < 1){
        ROS_INFO("The safety distance is too small !");
        std::cout << "The safety distance is too small !" << std::endl;
        return false;
    }

    geometry_msgs::PoseStamped temp_point;         //当前点
    geometry_msgs::PoseStamped goal_point;           //goal点
    geometry_msgs::PoseStamped start_point;           //start点

    unsigned int x_min;   //保持安全距离，cell的x最小值
    unsigned int x_max;   //保持安全距离，cell的x最大值
    unsigned int y_min;   //保持安全距离，cell的y最小值
    unsigned int y_max;   //保持安全距离，cell的y最大值
    unsigned int cell_x;    //单位值m，转换到地图下的像素坐标cell_x
    unsigned int cell_y;    //单位值m，转换到地图下的像素坐标cell_y
    
    //循环变量path中index附近的点，分别取idx_后10个点，idx_前5个点
    if((pure_pursuit_->idx_ + number_greater_index_) < path_.poses.size()){
        cycle_end_index_ = pure_pursuit_->idx_ + number_greater_index_;
    }else{
        cycle_end_index_ = path_.poses.size();
    }
    if((pure_pursuit_->idx_ - number_less_index_) > 0){
        //cout << "pure_pursuit_->idx_ - 12 =" << pure_pursuit_->idx_ - 12 << endl;
        cycle_start_index_ = pure_pursuit_->idx_ - number_less_index_;
    }else{
        cycle_start_index_ = 0;
    }

    for(int index = cycle_start_index_; index < cycle_end_index_; index++){
        temp_point = path_.poses[index];

        my_costmap_->worldToMap(temp_point.pose.position.x, temp_point.pose.position.y, cell_x, cell_y);
        x_min = (cell_x > safe_cell) ? cell_x - safe_cell : cell_x;
        x_max = (cell_x + safe_cell < my_costmap_->getSizeInCellsX()) ? cell_x + safe_cell : cell_x;
        y_min = (cell_y > safe_cell) ? cell_y - safe_cell : cell_y;
        y_max = (cell_y +safe_cell < my_costmap_->getSizeInCellsY()) ? cell_y + safe_cell : cell_y;

        //在安全距离范围内查找cell的cost值
        for(unsigned int j = x_min; j < x_max; j++){
            for(unsigned int k = y_min; k < y_max; k++){
                if(my_costmap_->getCost(j, k) == costmap_2d::LETHAL_OBSTACLE ||
                    my_costmap_->getCost(j, k) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){

                    if(pub_vel_thread_flag_ && path_init_flag_){
                        pure_pursuit_->interrupt_request_ = true;                                 //中断pure_pursuit
                        pause_voice_flag = true;
                        cout << "SLEEP ......" << endl;
                        sleep(5.0);                                                                                                  //遇到障碍物停车等待5s，同时等待纯路径跟踪线程响应
                        ros::spinOnce();
                    }
                    pause_voice_flag = false;
                    int start_index = index;                                                                        //当前开始点start_index大小为index
                    int temp_index = -1;                                                                              //临时变量，接收FindMinIndex()返回的占据index
                    if(FindMinIndex(cycle_start_index_, cycle_end_index_, temp_index)){       //查询一下当前轨迹最小占据index，赋值给start_index
                        if(temp_index < start_index){
                            start_index = temp_index;
                        }
                        start_waypoint_index_ = (start_index > reduce_start_points_) ? start_index - reduce_start_points_ : 0;
                        avoid_flag_ = true;
                        break;
                    }
                }
            }
            if(avoid_flag_ == true){
                cout << "Temp Point Index : " << index << endl;
                cout << "Start Waypoint Index : " << start_waypoint_index_ << endl;
                break;
            }
        }

        if(avoid_flag_){
            avoid_flag_ = false;
            bool found_path = false;
            int count = 0;                                                                              //目标点超过size的次数
            int goal_count = 0;                                                                   //目标点扩大次数

            //该循环查找目标点，goal_waypoint_index
            for(int i = first_search_waypoints_delta_; i < search_waypoints_size_; i += search_waypoints_delta_){
                goal_waypoint_index_ = start_waypoint_index_ + i;
    
                if(goal_waypoint_index_ >= path_.poses.size()){
                    if(count > 0){
                        cout << "goal_waypoint_index >= path_.poses.size()" << endl;
                        break;
                    }
                    goal_waypoint_index_ = path_.poses.size() - 1;   //当第一次超过目标点时，将path最后一个点作为目标点
                    count++;
                }

                start_point = path_.poses[start_waypoint_index_];   //初始点
                goal_point = path_.poses[goal_waypoint_index_];    //目标点

                cout << "CycleFixPath : planner_->makePlan ......" << endl;

                vector<geometry_msgs::PoseStamped> global_plan;
                found_path = planner_->makePlan(start_point, goal_point, global_plan);

                if(found_path && !global_plan.empty()){
                    end_of_avoid_index = goal_waypoint_index_;
                    MergeAvoidWaypoints(global_plan, end_of_avoid_index);
                    if(path_.poses.size() > 0){
                        //ROS_INFO("Found GOAL at index = %d", goal_waypoint_index_);
                        cout << "Goal Waypoint Index : " << goal_waypoint_index_ << endl;
                        fix_path_ = true;
                        break;
                    }else{
                        found_path = false;
                    }
                }
                else{
                    if(start_waypoint_index_ > cycle_reduce_start_points_){
                        start_waypoint_index_ = start_waypoint_index_ - cycle_reduce_start_points_;
                    }
                }

            }

            if(fix_path_ == true){
                fix_path_ = false;
                //ROS_INFO("Fix path is TRUE !!! ");
                std::cout << "Fix path is TRUE !!!" << std::endl;
            }else{
                //ROS_ERROR("Can't find goal ,Fix path is FLASE !!!");
                cout << "Can't find goal, Fix path is FALSE !!!" << endl;
                return false;                         //当规划路径失败时，返回false
            }

        }
        
    }

    // if(pause_flag == true){
    //     //暂停状态下，若轨迹循环一遍后触发纯路径跟踪开始
    //     pub_vel_thread_flag_ = true;                                        //开始pure_pursuit
    // }
    if(!stop_flag && !cancel_flag){
        pub_vel_thread_flag_ = true; 
    }

    return true;
}

//查找最小start_waypoints_index，输入开始点、结束点，输出轨迹中障碍物最小索引
bool CleanAvoidance::FindMinIndex(const int& start, const int& end, int& temp_index){
    if(path_.poses.empty()){
        ROS_INFO("FindMinIndex: plan is EMPTY!");
        return false;
    }

    int safe_cell = (int)( safe_distance_/my_costmap_->getResolution() );

    if(safe_cell < 1){
        cout << "FindMinIndex: The safety distance is too small !" << endl;
        return false;
    }

    geometry_msgs::PoseStamped temp_point;              //当前点
    geometry_msgs::PoseStamped goal_point;           //goal点
    geometry_msgs::PoseStamped start_point;           //start点

    unsigned int x_min;   //保持安全距离，cell的x最小值
    unsigned int x_max;   //保持安全距离，cell的x最大值
    unsigned int y_min;   //保持安全距离，cell的y最小值
    unsigned int y_max;   //保持安全距离，cell的y最大值
    unsigned int cell_x;    //单位值m，转换到地图下的像素坐标cell_x
    unsigned int cell_y;    //单位值m，转换到地图下的像素坐标cell_y
    
    //循环变量path中的全部点，查找当前point在安全距离范围内是否存在占据状态的cell
    for(int index = start; index < end; index++){
        temp_point = path_.poses[index];

        my_costmap_->worldToMap(temp_point.pose.position.x, temp_point.pose.position.y, cell_x, cell_y);
        x_min = (cell_x > safe_cell) ? cell_x - safe_cell : cell_x;
        x_max = (cell_x + safe_cell < my_costmap_->getSizeInCellsX()) ? cell_x + safe_cell : cell_x;
        y_min = (cell_y > safe_cell) ? cell_y - safe_cell : cell_y;
        y_max = (cell_y +safe_cell < my_costmap_->getSizeInCellsY()) ? cell_y + safe_cell : cell_y;

        //在安全距离范围内查找cell的cost值
        for(unsigned int j = x_min; j < x_max; j++){
            for(unsigned int k = y_min; k < y_max; k++){
                    if(my_costmap_->getCost(j, k) == costmap_2d::LETHAL_OBSTACLE ||
                        my_costmap_->getCost(j, k) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                    //start_waypoint_index_ = index;
                    temp_index = index;
                    return true;
                }
            }
        }

    }

    return false;
}

void CleanAvoidance::MergeAvoidWaypoints(const vector<geometry_msgs::PoseStamped>& avoid_path, int& end_of_avoid_index){
    nav_msgs::Path temp_path = path_;

    //reset
    path_.poses.clear();

    if(start_waypoint_index_ > 0){
        //添加start_waypoint_index_前的path_点
        for(int m = 0; m < start_waypoint_index_; m++){
            path_.poses.push_back(temp_path.poses[m]);
        }
    }

    for(int i = 0; i < avoid_path.size(); i++){
        path_.poses.push_back(avoid_path[i]);
    }

    if(end_of_avoid_index < temp_path.poses.size() - 1){
        for(int m = end_of_avoid_index + 1; m < temp_path.poses.size(); m++){
            path_.poses.push_back(temp_path.poses[m]);
        }
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "clean_avoidance_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Warn);
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    vector<vector<double>> temp_points{{-4.58, -7.15}, {-5.48, 1.14}, {-3.59, 3.32},{0.45, 0}};
    //vector<vector<double>> temp_points{{0.45, 0}, {-3.59, 3.32}, {-5.48, 1.14},{-4.58, -7.15}};
    CleanAvoidance clean_avoidance(buffer);

    // while(ros::ok()){
    //     if(clean_avoidance.Init(temp_points)){
    //         clean_avoidance.Run();
    //         temp_points.clear();
    //     }else{
    //         cout << "BREAK !" << endl;
    //     }
    // }

    ros::spin();

    return 0;
}
