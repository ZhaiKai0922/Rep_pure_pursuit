#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>//完成机器人当前位置的监听
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <inspur_msgs/ReachGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <inspur_msgs/MoveToGoal.h>
#include <inspur_msgs/MapManager.h>
//#include <opencv2/opencv.hpp>//自动路径需求Mat类转换
//#include <opencv/core.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include "pure_pursuit.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include <map>
#include <cmath>
#include "inspur_msgs/MoveToGoal.h"
#include "inspur_msgs/TaskManager.h"
#include "inspur_msgs/TaskResult.h"
/**
 * @brief  清洁机器人路径任务管理类
 * 
 */
namespace task_manager{

        enum PathType{    //枚举常量，即HAND_DRAWN_PATH = 1，TEACHING_PATH = 2，AUTO_PATH = 3
                HAND_DRAWN_PATH = 1,
                TEACHING_PATH ,
                AUTO_PATH 
        };

        enum RobotCleanStatus{  //枚举常量，START = 1，SUSPEND = 2，PROCEED = 3，CANCEL = 4
                START = 1,
                SUSPEND,
                PROCEED,
                CANCEL
        };

        enum TaskResult{
                SERVICE_NORMAL_COMMOND =101011200,
                SERVICE_PARAM_EXCEPTION, 
                SERVICE_CANCEL_COMMOND,
                SERVICE_SUSPEND_COMMOND,
                SERVICE_PROCEED_COMMOND,
                SERVICE_FINISH_COMMOND
        };

        struct TaskResultReq{
                std::string map_name;
                int path_type;
                int path_id;
                int clean_mode;
                int task_result;
                int status;
        };

        typedef struct{
                std::string robot_sn;
                std::string mapname;
                int path_type;
                int path_id;
                int clean_mode;
                bool clean_en;
                int status;
                int task_id;
                std::string timestamp;
        }CleanTask;



        class TaskManager{
                public:
                        TaskManager(ros::NodeHandle &nh);
                        //TaskManager(ros::NodeHandle &nh,tf2_ros::Buffer &tf);
                        ~TaskManager();
                        void Initialize();
                        void Run();
                private:
                        //基本控制功能
                        void Start();
                        void Cancel();
                        void Suspend();
                        void Proceed();
                        bool SetCleanMode(int clean_mode,bool work_startup);
                        //消息接收
                        bool CleanTaskSrvCallback(inspur_msgs::TaskManager::Request &req,inspur_msgs::TaskManager::Response &res);
                        //消息反馈  
                        void SendTaskResult(TaskResultReq& task_result_);
                        //功能函数
                        void GetCurrentPose(geometry_msgs::Pose &current_pose);
                        bool Get_plan_path(std::vector<geometry_msgs::Pose> &robot_nav_plan);
                        int Get_current_path_type();
                        bool Read_json_path(string current_map_name,int path_type,int path_id);
                        bool AutoNavTypeSwitch(geometry_msgs::Pose nav_first_point);
                        float DistancePoints(geometry_msgs::Pose first_point, geometry_msgs::Pose second_point){
                                return std::sqrt(std::pow(first_point.position.x-second_point.position.x,2)+std::pow(first_point.position.y-second_point.position.y,2));
                        }
                        void publishZeroVelocity();
                        void CleanRobotWork();
                        bool FinishRobotCleanWork(bool work_startup);
                        bool pose2vector2d(std::vector<std::vector<double>> &points);
                private:
                        ros::NodeHandle TaskManagerNh_;
                        ros::Publisher PlanPublisher_;
                        ros::Publisher PassedPlanPublisher_;
                        ros::Publisher CmdvelPublisher_;
                        ros::ServiceServer PlanTaskRecieveSrv_;//task任务接收服务
                        ros::ServiceServer PlanTaskArea_;//task获取清洁服务面积
                        ros::ServiceClient ClientTargetGoal;//直接调用DWA完成点到点路径规划
                        ros::ServiceClient TaskResultclient_;//task结果反馈

                private:
                        std::vector<geometry_msgs::Pose> nav_path;
                        std::map<int,geometry_msgs::Pose> auto_nav_path;
                        int path_traj_index;//实时记录机器人执行任务的索引，用于恢复机器人导航状态
                        geometry_msgs::Pose current_robot_pose;
                        //tf2_ros::Buffer tf_;
                        costmap_2d::Costmap2DROS* global_costmap_ros;
                        CleanTask robot_clean_task;
                        TaskResultReq cur_task_result;
                        boost::shared_ptr<boost::thread> CleanRobotPlanThread_;  
                        bool task_execut_permission = false;
                        bool given_path = false;
                private:
                        boost::shared_ptr<PurePursuit> PurePursuit_;
};
}

#endif