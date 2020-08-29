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

class PathNode {
public:
    Path path_;
    double x_;
    double y_;
    double theta_;
    double length_;
    double heuristics_value_;
    PathNode* father_PathNode_;

    PathNode() {};
    ~PathNode() {};
    PathNode(Path path, double heuristics_value, PathNode* father_PathNode) {
        if(path.path_.size() == 0) {
            cout << "PathNode is error!" << endl;
        }
        x_ = path.path_[path.path_.size() - 1].x_;
        y_ = path.path_[path.path_.size() - 1].y_;
        theta_ = path.path_[path.path_.size() - 1].theta_;
        if(father_PathNode == NULL) {
            length_ = path.length_;
        }
        else { 
            length_ = path.length_ + father_PathNode->length_;
        }
        heuristics_value_ = heuristics_value;
        father_PathNode_ = father_PathNode;
        path_ = path;
    };

    bool operator<(PathNode* path_node) {
        return heuristics_value_ < path_node->heuristics_value_;
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
    vector<PathNode*> tree_;
    vector<PathNode*> open_list_;
    int obstacle_threshold_;
    double extend_dist_;
    vector<double> w_seq_;
    bool find_path_flag_;
    GeneratePaths generate_paths_;

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool nodeEquality(Node* node1, Node* node2);
    bool searchThePath();
    bool nodeObstacleCheck(Node* node);
    bool nodeObstacleCheck(double x, double y);
    void extendTree(Node* node);
    void extendTreeRoot();
    int beInTree(Node* node, vector<Node*>& tree);
    double deltaAngle(double angle0, double angle1);
    double vectorProgection(double base_x, double base_y, double x, double y);
    bool findPathCheck(double x, double y);
    void displayTheTree();
    void displayThePath();
    int bestSearchNode();
};

#endif
