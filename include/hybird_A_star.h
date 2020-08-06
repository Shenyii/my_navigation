#ifndef HYBIRD_A_STAR_H_
#define HYBIRD_A_STAR_H_

#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265

using namespace std;

class Node {
public:
    double x_;
    double y_;
    double theta_;
    double v_;
    double w_;
    Node* father_node_;
    double heuristics_value_;

    Node();
    ~Node() {};
    Node(double x, double y, double theta) {
        x_ = x;
        y_ = y;
        theta_ = theta;
        heuristics_value_ = 0;
    };
    Node(double x, double y, double theta, double v, double w, double heuristics_value) {
        x_ = x;
        y_ = y;
        theta_ = theta;
        v_ = v;
        w_ = w;
        heuristics_value_ = heuristics_value;
    };

    bool operator<(Node* node) {
        return heuristics_value_ < node->heuristics_value_;
    };
};

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

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool nodeEquality(Node* node1, Node* node2);
    bool searchThePath();
    bool nodeObstacleCheck(Node* node);
    bool nodeObstacleCheck(double x, double y);
    void extendTree(Node* node);
    bool beInTree(Node* node, vector<Node*>& tree);
};

#endif
