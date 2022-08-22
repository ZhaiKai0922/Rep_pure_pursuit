//通过点生成直线
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <tf/transform_listener.h>

using namespace std;

class PubPath{
    public:
    PubPath();
    void ReceivePoints(const vector<vector<double>>& points);  //将私有成员points_进行赋值
    void MakeLine(const vector<double>& first, const vector<double>& second);   //通过一个点制作一条直线
    void LineVisual();
    void Run(const vector<vector<double>>& points);
    void ResetPath();
    nav_msgs::Path path_;

    private:
    vector<vector<double>> points_;
    ros::Publisher path_pub_;
    tf::TransformListener listener_;
    vector<double> robot_point_{0.0, 0.0};
    ros::NodeHandle nh_;
    double pathResolution_;
    double first_time = true;
};