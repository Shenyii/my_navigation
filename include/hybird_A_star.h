#ifndef HYBIRD_A_STAR_H_
#define HYBIRD_A_STAR_H_

#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define PI 3.14159265

using namespace std;

class HybirdAStar {
public:
    HybirdAStar();
    ~HybirdAStar();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_ori_map_;
    nav_msgs::OccupancyGrid ori_map_;
    ros::Subscriber sub_start_pose_;

    void subOriMap(nav_msgs::OccupancyGrid map);
    void subStartPose(geometry_msgs::PoseWithCovarianceStamped start_pose);

    double start_x_;
    double start_y_;
    double start_theta_;
    double goal_x_;
    double goal_y_;
    double goal_theta_;

    void worldToMap(double wx, double wy, int& mx, int& my);
};

#endif
