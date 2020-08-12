#include "hybird_A_star.h"

HybirdAStar::HybirdAStar()
: start_x_(0), start_y_(0), start_theta_(0), obstacle_threshold_(65), find_path_flag_(0) {
    sub_ori_map_ = nh_.subscribe("/map", 2, &HybirdAStar::subOriMap, this);
    sub_start_pose_ = nh_.subscribe("/initialpose", 2, &HybirdAStar::subStartPose, this);
    sub_goal_pose_ = nh_.subscribe("/move_base_simple/goal", 2, &HybirdAStar::subGoalPose, this);
    pub_tree_ = nh_.advertise<sensor_msgs::PointCloud>("/tree", 3);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/own_path", 3);
    w_seq_.push_back(20);
    w_seq_.push_back(10);
    w_seq_.push_back(5);
    w_seq_.push_back(0);
    w_seq_.push_back(-5);
    w_seq_.push_back(-10);
    w_seq_.push_back(-20);
    ros::Duration(1).sleep();

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
    extend_dist_ = ori_map_.info.resolution * 1.414;
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
    //cout << goal_x_ << ", " << goal_y_ << ", " << goal_theta_ << endl;
    searchThePath();
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
    // ///////////////////////
    // start_x_ = 0;
    // start_y_ = 0.5;
    // goal_x_ = 1.5;
    // goal_y_ = 1.5;
    // ///////////////////////
    if(nodeObstacleCheck(start_x_, start_y_) || nodeObstacleCheck(goal_x_, goal_y_)) {
        cout << "the start or goal is wrong!" << endl;
        return false;
    }
    for(int i = 0; i < tree_.size(); i++) {
        delete tree_[i];
    }
    tree_.clear();
    tree_.push_back(new Node(start_x_, start_y_, start_theta_, 1, 0, 0));
    tree_[0]->father_node_ = NULL;
    open_list_.clear();
    open_list_.push_back(tree_[0]);
    while(ros::ok() && open_list_.size() > 0 && find_path_flag_ == false) {
        int index = bestSearchNode();
        // for(int i = 0; i < open_list_.size(); i++) {
        //     cout << open_list_[i]->heuristics_value_ << endl;
        // }
        // cout << index << ", " << open_list_[index]->heuristics_value_ << endl;
        // getchar();
        extendTree(open_list_[index]);
        open_list_.erase(open_list_.begin() + index, open_list_.begin() + index + 1);
        //cout << "test: " << open_list_.size() << endl;
    }

    if(find_path_flag_ == true) {
        cout << "find a path." << endl;
        find_path_flag_ = false;
        displayThePath();
    }
    else {
        cout << "Not find a path." << endl;
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
    int test = ori_map_.data[mx + my * ori_map_.info.width];
    if(ori_map_.data[mx + my * ori_map_.info.width] < obstacle_threshold_ && ori_map_.data[mx + my * ori_map_.info.width] != -1) {
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
        double heuristics_value = 0;
        //heuristics_value += node->heuristics_value_;
        // heuristics_value += fabs(w) * det_t / PI;
        // heuristics_value -= vectorProgection(goal_x_ - node->x_, goal_y_ - node->y_, x - node->x_, y - node->y_);
        heuristics_value += hypot(x - goal_x_, y - goal_y_);
        //heuristics_value += fabs(w - node->w_);
        if(nodeObstacleCheck(x, y) == true) {
            continue;
        }
        Node* new_node;
        new_node = new Node(x, y, theta, v, w, heuristics_value);
        new_node->father_node_ = node;
        double index = beInTree(new_node, tree_);
        if(index != -1) {
            if(new_node->heuristics_value_ < tree_[index]->heuristics_value_) {
                tree_[index]->copy(new_node);
                displayTheTree();
                findPathCheck(x, y);
            }
            delete new_node;
        }
        else {
            tree_.push_back(new_node);
            displayTheTree();
            open_list_.push_back(new_node);
            findPathCheck(x, y);
        }
    }
}

int HybirdAStar::beInTree(Node* node, vector<Node*>& tree) {
    for(int i = 0; i < tree.size(); i++) {
        if(nodeEquality(node, tree[i])) {
            return i;
        }
    }
    return -1;
}

double HybirdAStar::deltaAngle(double angle0, double angle1) {
    double ans;
    ans = angle1 - angle0;
    ans = ans < -PI ? ans + 2 * PI : ans;
    ans = ans > PI ? ans - 2 * PI : ans;

    return ans;
}

double HybirdAStar::vectorProgection(double base_x, double base_y, double x, double y) {
    return (base_x * x + base_y * y) / hypot(base_x, base_y);
}

bool HybirdAStar::findPathCheck(double x, double y) {
    int x0;
    int y0;
    int x1;
    int y1;
    worldToMap(x, y, x0, y0);
    worldToMap(goal_x_, goal_y_, x1, y1);
    if((x0 == x1) && (y0 == y1)) {
        find_path_flag_ = true;
        return true;
    }
    return false;
}

void HybirdAStar::displayTheTree() {
    sensor_msgs::PointCloud points;
    points.header.frame_id = "map";
    for(int i = 1; i < tree_.size(); i++) {
        double v = 1.0;
        double det_t = extend_dist_ / v;
        double w = (tree_[i]->theta_ - tree_[i]->father_node_->theta_) / det_t;
        double x;
        double y;
        double theta;
        for(double t = 0; t < det_t; t+=0.007) {
            geometry_msgs::Point32 point;
            point.x = x;
            point.y = y;
            point.z = 0.02;
            points.points.push_back(point);
        }
    }
    pub_tree_.publish(points);
    //getchar();
}

void HybirdAStar::displayThePath() {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    Node* path_node = tree_[tree_.size() - 1];
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    while(path_node->father_node_ != NULL) {
        pose.pose.position.x = path_node->x_;
        pose.pose.position.y = path_node->y_;
        pose.pose.position.z = 0.05;
        path_node = path_node->father_node_;
        path.poses.push_back(pose);
    }

    pub_path_.publish(path);
    cout << "display the path." << endl;
}

int HybirdAStar::bestSearchNode() {
    int index;
    double value = 9999999999;
    for(int i = 0; i < open_list_.size(); i++) {
        if(value > open_list_[i]->heuristics_value_) {
            value = open_list_[i]->heuristics_value_;
            index = i;
        }
    }
    return index;
}

int main(int argc, char** argv) {
    cout << "begin the program." << endl;
    ros::init(argc, argv, "hybird_A_star");
    //HybirdAStar test;
    GeneratePaths test2;
    return 0;
}