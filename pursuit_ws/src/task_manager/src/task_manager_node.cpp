#include <ros/ros.h>
#include "task_manager.h"
//#include <tf2_ros/transform_listener.h>
int main(int argc, char **argv) 
{
		ros::init(argc, argv, "task_manager_node");
		//tf2_ros::Buffer buffer(ros::Duration(10));
    	//tf2_ros::TransformListener tf(buffer);
		ros::NodeHandle nh("task_manager_node");
	    task_manager::TaskManager Task(nh);
		Task.Initialize();
		Task.Run();

		return 0;
} 