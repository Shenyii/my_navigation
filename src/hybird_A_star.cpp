#include "hybird_A_star.h"

HybirdAStar::HybirdAStar()
: start_x_(0), start_y_(0), start_theta_(0), obstacle_threshold_(65) {
    sub_ori_map_ = nh_.subscribe("/map", 2, &HybirdAStar::subOriMap, this);
    sub_start_pose_ = nh_.subscribe("/initialpose", 2, &HybirdAStar::subStartPose, this);
    sub_goal_pose_ = nh_.subscribe("/move_base_simple/goal", 2, &HybirdAStar::subGoalPose, this);
    w_seq_.push_back(20);
    w_seq_.push_back(10);
    w_seq_.push_back(5);
    w_seq_.push_back(0);
    w_seq_.push_back(-5);
    w_seq_.push_back(-10);
    w_seq_.push_back(-20);
    ros::Duration(1).sleep();
    extend_dist_ = ori_map_.info.resolution * 1.414;

    while(ros::ok()) {
        ros::spin();
    }

    ///////////////////////
    int mx;
    int my;
    worldToMap(-6, -5, mx, my);
    cout << mx << ", " << my << endl;
    ///////////////////////
}

HybirdAStar::~HybirdAStar() {}

void HybirdAStar::subOriMap(nav_msgs::OccupancyGrid map) {
    ori_map_ = map;
    cout << "get the origin map." << endl;
}

void HybirdAStar::subStartPose(geometry_msgs::PoseWithCovarianceStamped start_pose) {
    start_x_ = start_pose.pose.pose.position.x;
    start_y_ = start_pose.pose.pose.position.y;
    start_theta_ = acos(2 * pow(start_pose.pose.pose.orientation.w, 2) - 1);
    start_theta_ = start_pose.pose.pose.orientation.w * start_pose.pose.pose.orientation.z < 0 ? - start_theta_ : start_theta_;
    //cout << start_x_ << ", " << start_y_ << ", " << start_theta_ << endl;
}

void HybirdAStar::subGoalPose(geometry_msgs::PoseStamped goal_pose) {
    goal_x_ = goal_pose.pose.position.x;
    goal_y_ = goal_pose.pose.position.y;
    goal_theta_ = acos(2 * pow(goal_pose.pose.orientation.w, 2) - 1);
    goal_theta_ = goal_pose.pose.orientation.w * goal_pose.pose.orientation.z < 0 ? - goal_theta_ : goal_theta_;
    cout << goal_x_ << ", " << goal_y_ << ", " << goal_theta_ << endl;
}

void HybirdAStar::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = (wx - ori_map_.info.origin.position.x) / ori_map_.info.resolution;
    my = (wy - ori_map_.info.origin.position.y) / ori_map_.info.resolution;
}

bool HybirdAStar::nodeEquality(Node* node1, Node* node2) {
    int mx1;
    int my1;
    int mx2;
    int my2;
    worldToMap(node1->x_, node1->y_, mx1, my1);
    worldToMap(node2->x_, node2->y_, mx2, my2);
    if((mx1 == mx2) && (my1 == my2)) {
        return true;
    }
    else {
        return false;
    }
}

bool HybirdAStar::searchThePath() {
    bool ans;
    if(nodeObstacleCheck(start_x_, start_y_) || nodeObstacleCheck(goal_x_, goal_y_)) {
        return false;
    }
    for(int i = 0; i < tree_.size(); i++) {
        delete tree_[i];
    }
    tree_.clear();
    tree_.push_back(new Node(start_x_, start_y_, start_theta_, 1, 0, 0));
    open_list_.clear();
    open_list_.push_back(tree_[0]);
    while(ros::ok() && open_list_.size() > 0) {
        sort(open_list_.begin(), open_list_.end());
        extendTree(open_list_[0]);
    }

    return false;
}

bool HybirdAStar::nodeObstacleCheck(Node* node) {
    int mx;
    int my;
    worldToMap(node->x_, node->y_, mx, my);
    if(ori_map_.data[mx + my * ori_map_.info.width] < obstacle_threshold_) {
        return false;
    }
    else {
        return true;
    }
}

bool HybirdAStar::nodeObstacleCheck(double x, double y) {
    int mx;
    int my;
    worldToMap(x, y, mx, my);
    if(ori_map_.data[mx + my * ori_map_.info.width] < obstacle_threshold_) {
        return false;
    }
    else {
        return true;
    }
}

void HybirdAStar::extendTree(Node* node) {
    double v = 1;
    double det_t = extend_dist_ / v;

    for(int i = 0; i < w_seq_.size(); i++) {
        double w = w_seq_[i];
        double theta = node->theta_ + w * det_t;
        double x = node->x_ + extend_dist_ * cos(theta);
        double y = node->y_ + extend_dist_ * sin(theta);
        double heuristics_value;
    }
}

bool HybirdAStar::beInTree(Node* node, vector<Node*>& tree) {
    for(int i = 0; i < tree.size(); i++) {
        if(nodeEquality(node, tree[i])) {
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    cout << "begin the program." << endl;
    ros::init(argc, argv, "hybird_A_star");
    HybirdAStar test;
    return 0;
}