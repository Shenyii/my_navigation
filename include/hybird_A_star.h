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
#include "opt_quintic_curve.h"

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

    bool operator==(PathNode* path_node) {
        if((x_ == path_node->x_) && (y_ == path_node->y_)) {
            return true;
        }
        else {
            return false;
        }
    };
};

bool pathNodeLess(PathNode* path_node1, PathNode* path_node2) {
    return path_node1->heuristics_value_ < path_node2->heuristics_value_;
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
    bool find_path_flag_;

    GeneratePaths generate_paths_;

    OptQuinticCurve opt_quintic_curve_;

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool searchThePath();
    bool nodeObstacleCheck(double x, double y);
    void extendTree();
    void extendTreeRoot();
    int beInList(PathNode* path_node, vector<PathNode*> tree);
    int findPath();
    void displayTheTree();
    void displayThePath();
    void add2Openlist(PathNode* path_node);
};

#endif
