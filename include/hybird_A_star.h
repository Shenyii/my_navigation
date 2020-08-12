#ifndef HYBIRD_A_STAR_H_
#define HYBIRD_A_STAR_H_

#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>

#include "generate_paths.h"

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
    ros::Subscriber sub_goal_pose_;
    ros::Publisher pub_tree_;
    ros::Publisher pub_path_;

    void subOriMap(nav_msgs::OccupancyGrid map);
    void subStartPose(geometry_msgs::PoseWithCovarianceStamped start_pose);
    void subGoalPose(geometry_msgs::PoseStamped goal_pose);

    double start_x_;
    double start_y_;
    double start_theta_;
    double goal_x_;
    double goal_y_;
    double goal_theta_;
    vector<Node*> tree_;
    vector<Node*> open_list_;
    int obstacle_threshold_;
    double extend_dist_;
    vector<double> w_seq_;
    bool find_path_flag_;

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool nodeEquality(Node* node1, Node* node2);
    bool searchThePath();
    bool nodeObstacleCheck(Node* node);
    bool nodeObstacleCheck(double x, double y);
    void extendTree(Node* node);
    int beInTree(Node* node, vector<Node*>& tree);
    double deltaAngle(double angle0, double angle1);
    double vectorProgection(double base_x, double base_y, double x, double y);
    bool findPathCheck(double x, double y);
    void displayTheTree();
    void displayThePath();
    int bestSearchNode();
};

#endif
