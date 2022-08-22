#include "task_manager.h"
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <geometry_msgs/Twist.h>
namespace task_manager{
        TaskManager::TaskManager(ros::NodeHandle &nh):TaskManagerNh_(nh){

        }
       /* TaskManager::TaskManager(ros::NodeHandle &nh,tf2_ros::Buffer &tf):TaskManagerNh_(nh),tf_(tf){
        }*/
        TaskManager::~TaskManager(){

        }

        void TaskManager::Initialize(){
                ROS_INFO("Task_manager start initialize...");   
                //话题发布
                CmdvelPublisher_ = TaskManagerNh_.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
                PlanPublisher_ = TaskManagerNh_.advertise<nav_msgs::Path>("/clean_path",1,true);
                PassedPlanPublisher_ = TaskManagerNh_.advertise<nav_msgs::Path>("/cleaned_path",1,true);
                //服务发布
                PlanTaskRecieveSrv_ = TaskManagerNh_.advertiseService("/task_manager",&TaskManager::CleanTaskSrvCallback,this);
                //客户订阅TaskManager
                ClientTargetGoal = TaskManagerNh_.serviceClient<inspur_msgs::MoveToGoal>("/move_to_goal");
                TaskResultclient_ = TaskManagerNh_.serviceClient<inspur_msgs::TaskResult>("/task_result");
                //应用对象
                PurePursuit_ = boost::make_shared<PurePursuit>();
                //global_costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf_); 
                CleanRobotPlanThread_ = boost::make_shared<boost::thread>(boost::bind(&TaskManager::CleanRobotWork,this));
                ROS_INFO("Task_manager initialize completed...");
        }
        
        void TaskManager::Run(){
            ros::spin();
        }

        void TaskManager::Start(){
                    
                if(robot_clean_task.path_type == PathType::HAND_DRAWN_PATH){
                        if(robot_clean_task.clean_en)   
                                SetCleanMode(robot_clean_task.clean_mode,true);
                        //这里等待2s完成机器人扫地功能调整
                        ros::Duration(2).sleep();
                        if(nav_path.size()>=2){
                                std::vector<std::vector<double>> points(nav_path.size(),std::vector<double>(2,0));
                                pose2vector2d(points);
                                PurePursuit_->PathInit(points);
                                PurePursuit_->interrupt_request_ = false;
                                given_path = true;
                                task_execut_permission = true;  
                        }
                        else{
                                ROS_WARN("NAV PATH POINTS TOO FEW...");
                                given_path = false;
                        }
                        cur_task_result.task_result = SERVICE_NORMAL_COMMOND;
                }
                else if(robot_clean_task.path_type == PathType::TEACHING_PATH){

                }    
                else if(robot_clean_task.path_type == PathType::AUTO_PATH){

                }
                else{
                        ROS_ERROR("input param error");
                        cur_task_result.task_result = SERVICE_PARAM_EXCEPTION;
                }
                SendTaskResult(cur_task_result);
        }

        void TaskManager::Cancel(){
                
                if(robot_clean_task.path_type == PathType::HAND_DRAWN_PATH){
                        //publishZeroVelocity();
                        PurePursuit_->interrupt_request_ = true;
                        nav_path.clear();
                        PurePursuit_->ResetPath();
                        PurePursuit_->ResetID();
                        // PurePursuit_ -> work_status = false;  //发送中断
                        //PurePursuit_ 路径清空
                        cur_task_result.task_result = SERVICE_CANCEL_COMMOND;//任务取消
                }
                else if(robot_clean_task.path_type == PathType::TEACHING_PATH){

                }    
                else if(robot_clean_task.path_type == PathType::AUTO_PATH){

                }
                else{
                        ROS_ERROR("input param error");
                        cur_task_result.task_result = SERVICE_PARAM_EXCEPTION;
                }
                SendTaskResult(cur_task_result);
                //PurePursuit_的路径
        }

        void TaskManager::Suspend(){
                if(robot_clean_task.path_type == PathType::HAND_DRAWN_PATH){
                        //publishZeroVelocity();
                        PurePursuit_->interrupt_request_ = true;
                        cur_task_result.task_result = SERVICE_SUSPEND_COMMOND;
                }
                else if(robot_clean_task.path_type == PathType::TEACHING_PATH){

                }    
                else if(robot_clean_task.path_type == PathType::AUTO_PATH){

                }
                else{
                        ROS_ERROR("input param error");
                        cur_task_result.task_result = SERVICE_PARAM_EXCEPTION;
                }
                SendTaskResult(cur_task_result);
        }

        void TaskManager::Proceed(){
                if(robot_clean_task.path_type == PathType::HAND_DRAWN_PATH){
                        if(robot_clean_task.clean_en)   
                                SetCleanMode(robot_clean_task.clean_mode,true);
                        //这里等待2s完成机器人扫地功能调整
                        ros::Duration(2).sleep();
                        //PurePursuit_ ->reset_nav_begin(path_traj_index);

                          if(nav_path.size()>=2){
                                //std::vector<std::vector<double>> points(nav_path.size(),std::vector<double>(2,0));
                                //pose2vector2d(points);
                                //PurePursuit_->PathInit(points);
                                //这里暂时没有做任务跟进
                                given_path = true;
                                task_execut_permission = true;  
                        }
                        else{
                                ROS_WARN("NAV PATH POINTS TOO FEW...");
                                given_path = false;
                        }
                        cur_task_result.task_result = SERVICE_PROCEED_COMMOND;
                }
                else if(robot_clean_task.path_type == PathType::TEACHING_PATH){

                }    
                else if(robot_clean_task.path_type == PathType::AUTO_PATH){

                }
                else{
                        ROS_ERROR("input param error");
                        cur_task_result.task_result = SERVICE_PARAM_EXCEPTION;
                }
                SendTaskResult(cur_task_result);
        }

        //service回调处理
        bool TaskManager::CleanTaskSrvCallback(inspur_msgs::TaskManager::Request &req,inspur_msgs::TaskManager::Response &res){
               // robot_clean_task.robot_sn = req.robot_sn;
                robot_clean_task.mapname = req.map_name;
                robot_clean_task.path_type = req.path_type;
                robot_clean_task.path_id = req.path_id;
                robot_clean_task.clean_mode = req.clean_mode;
                robot_clean_task.clean_en = req.clean_en;
                robot_clean_task.status = req.status;
                //robot_clean_task.task_id = req.task_id;
               // robot_clean_task.timestamp = req.timestamp;

                cur_task_result.map_name = robot_clean_task .mapname;
                cur_task_result.path_type = robot_clean_task.path_type;
                cur_task_result.path_id = robot_clean_task.path_id;
                cur_task_result.clean_mode = robot_clean_task.clean_mode;
                cur_task_result.status = robot_clean_task.status;

                if(Read_json_path(req.map_name, req.path_type, req.path_id)){
                        ROS_INFO("CURRENT MAP: %s, PATH_TYPE: %d,PATH_ID:%d",req.map_name.c_str(),req.path_type,req.path_id);
                }
                else{
                        res.result = false;
                        return false;
                }
                
                switch(req.status){
                        case RobotCleanStatus::START:{     //req.status == 1
                                Start();
                                std::cout<<"开始"<<std::endl;
                                res.result = true;
                        }
                        break;
                        case RobotCleanStatus::SUSPEND:{
                                Suspend();
                                std::cout<<"暂停"<<std::endl;
                                res.result = true;
                        }
                        break;
                        case RobotCleanStatus::PROCEED:{
                                Proceed();
                                std::cout<<"继续"<<std::endl;
                                res.result = true;
                        }
                        break;
                        case RobotCleanStatus::CANCEL:{
                                Cancel();
                                std::cout<<"取消"<<std::endl;
                                res.result = true;
                        }
                        break;
                        default:
                                ROS_WARN("ERROR WORK STATUS,PLEASE CHECK");
                                res.result = false;
                                return false;
                                break;
                }
                return true;
        } 

        void TaskManager::GetCurrentPose(geometry_msgs::Pose &current_pose)
        {
                tf::TransformListener listener;
                //等待tf树系统中是否存在/map、/base_link这两个坐标系，存在的话才会跳到下一句，等待3秒之后就会提示超时，引发错误
                listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(4.0));
                tf::StampedTransform transform;
                try
                {
                        //获得当前时刻base_link相对于map的坐标转换关系
                        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform); //ros::Time(0)表示最近的一帧坐标变换，不能写成ros::Time::now()
                }
                catch (tf::TransformException& ex)
                {
                        //ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                }
                //由于tf会把监听的内容存放到一个缓存中，然后再读取相关的内容
                //而这个过程可能会有几毫秒的延迟，也就是，tf的监听器并不能监听到“现在”的变换
                //所以如果不使用try,catch函数会导致报错：
                current_pose.position.x = transform.getOrigin().x();
                current_pose.position.y = transform.getOrigin().y();
                current_pose.position.z = transform.getOrigin().z();
                current_pose.orientation.x = transform.getRotation().getX();
                current_pose.orientation.y = transform.getRotation().getY();
                current_pose.orientation.z = transform.getRotation().getZ();
                current_pose.orientation.w = transform.getRotation().getW();
        }
        
        //不知道这个workstartup workmode同时被设定有问题没，如work为false，mode被设定
        bool TaskManager::SetCleanMode(int clean_mode,bool work_startup){ 
                /*inspur_msgs::Drivecontrol msg;
                msg.request.workstartup = work_startup;
                msg.request.workmode = clean_mode;
                //这里服务也可以这样调用
                if(ros::service::call("/clean_mode_srv",msg))
                {
                        return true;
                }   
                else
                {         
                        ROS_ERROR("set clean mode fail!");
                        return false;     
                }*/
        }

        bool TaskManager::FinishRobotCleanWork(bool work_startup){
                /*
                inspur_msgs::Drivecontrol msg;
                msg.request.workstartup = work_startup;
                 if(ros::service::call("/clean_mode_srv",msg))
                {
                        return true;
                }   
                else
                {         
                        ROS_ERROR("set clean mode fail!");
                        return false;     
                }
                */
        }

        void TaskManager::SendTaskResult(TaskResultReq& task_result_)
        {
                inspur_msgs::TaskResult task_result;
                task_result.request.map_name = task_result_.map_name;
                task_result.request.path_type = task_result_.path_type;    
                task_result.request.path_id = task_result_.path_id;
                task_result.request.clean_mode = task_result_.clean_mode;
                task_result.request.task_result = task_result_.task_result;
                TaskResultclient_.call(task_result);
                if(task_result.response.response)
                        ROS_INFO("Send Task Result sucsess...");
                else
                        ROS_INFO("Send Task Result fail...");
        }
        
        bool TaskManager::AutoNavTypeSwitch(geometry_msgs::Pose nav_first_point){
                geometry_msgs::Pose current_pose;
                GetCurrentPose(current_pose);
                //这里先不考虑那个继续通过DWA通过的方式
                if(DistancePoints(current_pose,nav_first_point)>0.5 && robot_clean_task.status ==START){
                        inspur_msgs::MoveToGoal srv_move_to_goal;
                        srv_move_to_goal.request.type = 1;
                        srv_move_to_goal.request.map_name = robot_clean_task.mapname;
                        srv_move_to_goal.request.pose = nav_first_point;
                        if (ClientTargetGoal.call(srv_move_to_goal))
                                ROS_DEBUG("Success to call move_to_goal service!");
                        else
                                ROS_ERROR("Failed to call move_to_goal service!");
                        if(srv_move_to_goal.response.result != 101003200)
                                return false;
                }
                else if(robot_clean_task.status ==CANCEL||robot_clean_task.status ==PROCEED||robot_clean_task.status ==SUSPEND){
                        inspur_msgs::MoveToGoal srv_move_to_goal;
                        srv_move_to_goal.request.type = 3;
                        if (ClientTargetGoal.call(srv_move_to_goal))
                                ROS_DEBUG("Success to call move_to_goal service!");
                        else
                                ROS_ERROR("Failed to call move_to_goal service!");
                        if(srv_move_to_goal.response.result != 101003200)
                                return false;
                }
                return true;
                
        }

        bool TaskManager::Read_json_path(string current_map_name,int path_type,int path_id){
                Json::Value plan_value;  //用于接收信息
                Json::Reader reader;       //读取json
                std::string base_path = "/home/zk/zk/pure_pursuit/pursuit_ws/paths/";
                std::string path_paths = base_path+current_map_name+".json";
                std::ifstream in(path_paths.c_str(),std::ios::binary);
                if(reader.parse(in,plan_value))
                {   
			std::cout<<plan_value["map_name"].asString()<<" "<<current_map_name<<std::endl;
                        if(plan_value["map_name"].asString()!=current_map_name){
                                ROS_WARN("The current map does not match the file information...");
                                in.close();
                                return false;
                        }
                        int temp = -1;
                        for(int j = 0; j < plan_value["path_info"].size();j++)
                        {
                                if(plan_value["path_info"][j]["path_type"].asInt() == path_type )
                                {
                                        temp = j;
                                        break;
                                }
                        }
                        if(temp == -1)
                        { 
                                in.close();
                                return false;
                        }
                        int path_type_index = temp;
                        temp = -1;
                        for(int j = 0; j < plan_value["path_info"][path_type_index]["path_ids"].size();j++)
                        {
                                if(plan_value["path_info"][path_type_index]["path_ids"][j].asInt() == path_id )
                                {
                                        temp = j;
                                        break;
                                }
                        }
                        if(temp == -1)
                        { 
                                in.close();
                                return false;
                        }
                        int path_id_index = temp;
                        temp = -1;
                        //FIXME:
                        if(path_type == PathType::HAND_DRAWN_PATH){
                                if(plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"].size()<2){
                                        ROS_WARN("TOO FEW POINTS,CANNOT CONSTITUTE HAND_DRAWN NAV WORK");
                                        return false;
                                }                                
                                for(int i = 0;i<plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"].size();i++){
                                        if(plan_value["path_info"][path_type_index]["path"][path_id_index]["path_id"].asInt()!=path_id){
                                                ROS_WARN("PATH ID NOT MATCH");
                                                break;
                                        }
                                        geometry_msgs::Pose plan_point;
                                        plan_point.position.x=plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"][i]["position"]["x"].asDouble();
                                        plan_point.position.y=plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"][i]["position"]["y"].asDouble();
                                        plan_point.position.z=plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"][i]["position"]["z"].asDouble();
                                        plan_point.orientation.x=plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"][i]["orientation"]["x"].asDouble();
                                        plan_point.orientation.y=plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"][i]["orientation"]["y"].asDouble();
                                        plan_point.orientation.z=plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"][i]["orientation"]["z"].asDouble();
                                        plan_point.orientation.w=plan_value["path_info"][path_type_index]["path"][path_id_index]["poses"][i]["orientation"]["w"].asDouble();
                                        //std::cout<<"plan_point.position.x: "<<plan_point.position.x << std::endl;
                                        //std::cout<<"plan_point.position.y: "<<plan_point.position.y << std::endl;

                                        nav_path.push_back(plan_point); //存储导航点 
                                }
                        }
                        else if(path_type == TEACHING_PATH){

                        }
                        else{
                               
                        }
                        return true;
        }
        }

        void TaskManager::publishZeroVelocity(){
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                CmdvelPublisher_.publish(cmd_vel);
        }

        void TaskManager::CleanRobotWork(){
                while(ros::ok()){
                        if(given_path && task_execut_permission ){
                                // if(AutoNavTypeSwitch(nav_path.front()))
                                // ROS_INFO("ROBOT REACHE FIRST NAV NEAR...");
                                // else{
                                //         ROS_INFO("ROBOT  NAV FAIL...");
                                // }  
                                PurePursuit_->Run();
                                FinishRobotCleanWork(false);
                                if(PurePursuit_->goal_reached_){
                                        cur_task_result.task_result = SERVICE_FINISH_COMMOND;//TODO:这里需要修改
                                        SendTaskResult(cur_task_result);
                                        task_execut_permission = false;
                                        PurePursuit_->goal_reached_ = false;
					nav_path.clear();
                                }
                                else if(PurePursuit_->interrupt_request_){
                                        //path_traj_index =  PurePursuit_ -> get_current_traj_index; 这边需要记录一下索引
                                        task_execut_permission = false;
                                        PurePursuit_->interrupt_request_ = false;
                                        publishZeroVelocity();
                                }
                        }
                        //else 
                              //  ROS_INFO("ROBOT WAITING WORK...");
                }
        }

        bool TaskManager::pose2vector2d(std::vector<std::vector<double>> &points){
                for(int i = 0;i<nav_path.size();i++){
                        points[i][0] = nav_path[i].position.x;
                        points[i][1] = nav_path[i].position.y;
                }
                return true;
        }

        bool TaskManager::Get_plan_path(std::vector<geometry_msgs::Pose> &robot_nav_plan){
                if(robot_clean_task.path_type == PathType::HAND_DRAWN_PATH)
                        robot_nav_plan = nav_path;
                else if(robot_clean_task.path_type == PathType::TEACHING_PATH)
                {

                }
                else if(robot_clean_task.path_type == PathType::AUTO_PATH){

                }
                else{
                        ROS_WARN("ERROR PATH TYPE...");
                        return false;
                }
                return true;
         }

        int TaskManager::Get_current_path_type(){
                return robot_clean_task.path_type;
        }

};

