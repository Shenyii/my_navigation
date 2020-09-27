#ifndef OPT_PATH_H
#define OPT_PATH_H

#include <iostream>

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

#include "opt_bezier_curve.h"
#include "opt_quintic_curve.h"

using namespace std;

class Node {
public:
    double x_;
    double y_;
    double theta_;
    vector<double> x_list_;
    vector<double> y_list_;
    vector<double> theta_list_;
    double distance_;
    Node* father_node_;

    Node(double x, double y, double theta) {
        x_ = x;
        y_ = y;
        theta_ = theta;
        distance_ = 0;
        father_node_ = NULL;
    };

    ~Node() {};
};

class OptPath {
public:
    OptPath();
    ~OptPath();
    vector<geometry_msgs::Pose2D> optThePath();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_ori_map_;
    nav_msgs::OccupancyGrid ori_map_;
    ros::Subscriber sub_ori_path_;
    nav_msgs::Path ori_path_;
    ros::Publisher pub_path_;
    ros::Publisher pub_poses_;
    ros::Publisher pub_tree_;

    void subOriMap(nav_msgs::OccupancyGrid map);
    void subOriPath(nav_msgs::Path path);
    void displayPath();
    void displayTree();

    vector<geometry_msgs::Pose2D> path_;
    double obstacle_threshold_;
    double map_resolution_;
    vector<Node*> tree_;

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool nodeObstacleCheck(double x, double y);
    bool pathObstacleCheck(MyPath path);
    void sampleOptPath();
    void pathPartition();

    OptQuinticCurve opt_path_;
};

#endif