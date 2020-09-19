#ifndef A_STAR_H
#define A_STAR_H

#include <iostream>
#include <algorithm>

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>

#include "opt_bezier_curve.h"
#include "opt_quintic_curve.h"

using namespace std;

class Node {
public:
    double x_;
    double y_;
    double theta_;
    double distance_;
    double heuristics_value_;
    Node* father_node_;

    Node(double x, double y, double distance, double value, Node* father_node) {
        x_ = x;
        y_ = y;
        theta_ = 0;
        distance_ = distance;
        heuristics_value_ = value;
        father_node_ = father_node;
    };
};

bool nodeLess(Node* node1, Node* node2) {
    return node1->heuristics_value_ < node2->heuristics_value_;
};

class AStar {
public:
    AStar();
    ~AStar();

private:
    nav_msgs::Path path_;
    double start_x_;
    double start_y_;
    double goal_x_;
    double goal_y_;
    double extend_dist_;
    vector<Node*> close_list_;
    vector<Node*> open_list_;
    int find_path_flag_;
    double obstacle_threshold_;

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool searchThePath();
    bool nodeObstacleCheck(double x, double y);
    void extendCloseList();
    int beInList(Node* node, vector<Node*> list);
    int beInList(double x, double y, vector<Node*> list);
    int findPathCheck();
    void getThePath();
    void add2Openlist(Node* node);

    ros::NodeHandle nh_;
    ros::Subscriber sub_ori_map_;
    nav_msgs::OccupancyGrid ori_map_;
    ros::Subscriber sub_start_pose_;
    ros::Subscriber sub_goal_pose_;
    ros::Publisher pub_path_;
    ros::Publisher pub_points;

    void subOriMap(nav_msgs::OccupancyGrid map);
    void subStartPose(geometry_msgs::PoseWithCovarianceStamped start_pose);
    void subGoalPose(geometry_msgs::PoseStamped goal_pose);
    void displayCloseList();
};

#endif