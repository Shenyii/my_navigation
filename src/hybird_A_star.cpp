#include "hybird_A_star.h"

HybirdAStar::HybirdAStar()
: start_x_(0), start_y_(0), start_theta_(0), obstacle_threshold_(65), find_path_flag_(0) {
    sub_ori_map_ = nh_.subscribe("/map", 2, &HybirdAStar::subOriMap, this);
    sub_start_pose_ = nh_.subscribe("/initialpose", 2, &HybirdAStar::subStartPose, this);
    sub_goal_pose_ = nh_.subscribe("/move_base_simple/goal", 2, &HybirdAStar::subGoalPose, this);
    pub_tree_ = nh_.advertise<sensor_msgs::PointCloud>("/tree", 3);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/own_path", 3);
    extend_dist_ = 0.5;
    ros::Duration(1).sleep();

    while(ros::ok()) {
        ros::spin();
    }
}

HybirdAStar::~HybirdAStar() {}

void HybirdAStar::subOriMap(nav_msgs::OccupancyGrid map) {
    ori_map_ = map;
    //extend_dist_ = ori_map_.info.resolution * 1.414;
    cout << "get the origin map." << endl;
    generate_paths_.loadTheMap(map);
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
    for(int i = 0; i < tree_.size(); i++) {
        delete tree_[i];
    }
    tree_.clear();
    open_list_.clear();
    PathNode* one_path_node;
    Path path;
    path.path_.push_back(Node(start_x_, start_y_, start_theta_));
    path.length_ = 0;
    one_path_node = new PathNode(path, 0, NULL);
    tree_.push_back(one_path_node);
    if(searchThePath() == true) {
        cout << "Find the path." << endl;
    }
    else {
        cout << "Can't find the path." << endl;
    }

    displayTheTree();

    opt_quintic_curve_.optSolveQuinticCurve2(start_x_, start_y_, start_theta_, goal_x_, goal_y_, goal_theta_);
}

void HybirdAStar::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = (wx - ori_map_.info.origin.position.x) / ori_map_.info.resolution;
    my = (wy - ori_map_.info.origin.position.y) / ori_map_.info.resolution;
}

bool HybirdAStar::searchThePath() {
    //generate_paths_.generatePaths(start_x_, start_y_, goal_x_, goal_y_, ori_map_.info.resolution);
    //generate_paths_.generatePaths(start_x_, start_y_, start_theta_, goal_x_, goal_y_, ori_map_.info.resolution);
    Path path = generate_paths_.generatePaths(start_x_, start_y_, goal_x_, goal_y_, ori_map_.info.resolution);
    if(path.path_.size() != 0) {
        PathNode* one_path_node;
        one_path_node = new PathNode(path, 0, tree_[0]);
        tree_.push_back(one_path_node);
        displayTheTree();
        if(findPath() == 1) {
            return true;
        }
        return true;
    }
    extendTreeRoot();
    if(tree_.size() > 1) {
        //while(open_list_.size() != 0 && findPath() != 1) {
        for(int i = 0; i < 80; i++){
            extendTree();
            if(findPath() == 1) {
                return true;
            }
        }
        
    }
    else {
        return false;
    }

    return false;
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

void HybirdAStar::extendTree() {
    sort(open_list_.begin(), open_list_.end(), pathNodeLess);
    PathNode* father_node = open_list_[0];
    open_list_.erase(open_list_.begin(), open_list_.begin() + 1);
    // for(int i = 0; i < open_list_.size(); i++) {
    //     cout << open_list_[i]->x_ << ", " << open_list_[i]->y_ << ", " << open_list_[i]->heuristics_value_ << endl;
    // }

    Path path = generate_paths_.generatePaths(father_node->x_, father_node->y_, father_node->theta_, goal_x_, goal_y_, ori_map_.info.resolution);
    if(path.path_.size() != 0) {
        PathNode* one_path_node;
        one_path_node = new PathNode(path, 0, father_node);
        tree_.push_back(one_path_node);
        displayTheTree();
        return;
    }
    for(double det_theta = -PI / 3 * ori_map_.info.resolution / extend_dist_; det_theta <= PI / 3 * ori_map_.info.resolution / extend_dist_; det_theta += PI / 9 * ori_map_.info.resolution / extend_dist_) {
        PathNode* one_path_node;
        Path one_path;
        Node one_node;
        one_node.x_ = father_node->x_;
        one_node.y_ = father_node->y_;
        one_node.theta_ = father_node->theta_;
        for(int i = 0; i < extend_dist_ / ori_map_.info.resolution; i++) {
            one_node.theta_ += det_theta;
            one_node.x_ += ori_map_.info.resolution * cos(one_node.theta_);
            one_node.y_ += ori_map_.info.resolution * sin(one_node.theta_);
            if(nodeObstacleCheck(one_node.x_, one_node.y_) == 1) {
                one_path.path_.clear();
                break;
            }
            one_path.path_.push_back(one_node);
        }
        if(one_path.path_.size() == 0) {
            continue;
        }
        double h = father_node->length_ + extend_dist_ + hypot((one_path.path_.end() - 1)->x_ - goal_x_, (one_path.path_.end() - 1)->y_ - goal_y_);
        one_path_node = new PathNode(one_path, h, father_node);
        int index = beInList(one_path_node, tree_);
        if(index != -1) {
            if(tree_[index]->length_ > one_path_node->length_) {
                tree_[index]->father_PathNode_ = one_path_node->father_PathNode_;
                tree_[index]->heuristics_value_ = one_path_node->heuristics_value_;
                tree_[index]->length_ = one_path_node->length_;
                tree_[index]->path_ = one_path_node->path_;
                tree_[index]->x_ = one_path_node->x_;
                tree_[index]->y_ = one_path_node->y_;
                tree_[index]->theta_ = one_path_node->theta_;
                delete one_path_node;
            }
        }
        else {
            tree_.push_back(one_path_node);
            add2Openlist(one_path_node);
            // cout << "generate a node: " << tree_[tree_.size() - 1]->path_.path_.size() << endl;
            // for(int i = 0; i < tree_[tree_.size() - 1]->path_.path_.size(); i++) {
            //     cout << tree_[tree_.size() - 1]->path_.path_[i].x_ << ", " << tree_[tree_.size() - 1]->path_.path_[i].y_ << endl;
            // }
        }
    }
}

void HybirdAStar::extendTreeRoot() {
    for(double theta = -PI; theta < PI; theta += PI / 20) {
        PathNode* one_path_node;
        Path one_path;
        Node one_node;
        double c = cos(theta);
        double s = sin(theta);
        for(double dist = 0; dist < extend_dist_; dist += ori_map_.info.resolution) {
            one_node.x_ = tree_[0]->path_.path_[0].x_ + dist * c;
            one_node.y_ = tree_[0]->path_.path_[0].y_ + dist * s;
            one_node.theta_ = theta;
            if(nodeObstacleCheck(one_node.x_, one_node.y_) == true) {
                one_path.path_.clear();
                break;
            }
            else {
                one_path.path_.push_back(one_node);
            }
        }
        if(one_path.path_.size() == 0) {
            continue;
        }
        else {
            double h = tree_[0]->length_ + extend_dist_ + hypot((one_path.path_.end() - 1)->x_ - goal_x_, (one_path.path_.end() - 1)->y_ - goal_y_);
            one_path.length_ = extend_dist_;
            one_path_node = new PathNode(one_path, h, tree_[0]);
            tree_.push_back(one_path_node);
            add2Openlist(one_path_node);
        }
    }
}

int HybirdAStar::beInList(PathNode* path_node, vector<PathNode*> tree) {
    for(int i = 0; i < tree.size(); i++) {
        int x0;
        int y0;
        int x1;
        int y1;
        worldToMap(tree_[i]->x_, tree_[i]->y_, x0, y0);
        worldToMap(path_node->x_, path_node->y_, x1, y1);
        if((x0 == x1) && (y0 == y1)) {
            return i;
        }
    }
    return -1;
}

int HybirdAStar::findPath() {
    int x0;
    int y0;
    int x1;
    int y1;
    worldToMap(tree_[tree_.size() - 1]->x_, tree_[tree_.size() - 1]->y_, x0, y0);
    worldToMap(goal_x_, goal_y_, x1, y1);
    if(abs(x0 - x1) < 2 && abs(y0 - y1) < 2) {
        find_path_flag_ = true;
        displayThePath();
        return 1;
    }
    return 0;
}

void HybirdAStar::displayTheTree() {
    sensor_msgs::PointCloud points;
    points.header.frame_id = "map";
    for(int i = 1; i < tree_.size(); i++) {
        for(int j = 0; j < tree_[i]->path_.path_.size(); j++) {
            geometry_msgs::Point32 point;
            point.x = tree_[i]->path_.path_[j].x_;
            point.y = tree_[i]->path_.path_[j].y_;
            point.z = 0.01;
            points.points.push_back(point);
        }
    }
    pub_tree_.publish(points);
    //getchar();
}

void HybirdAStar::displayThePath() {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    PathNode* one_path_node = tree_[tree_.size() - 1];
    while(one_path_node != NULL) {
        for(int i = one_path_node->path_.path_.size() - 1; i >= 0; i--) {
            pose.pose.position.x = one_path_node->path_.path_[i].x_;
            pose.pose.position.y = one_path_node->path_.path_[i].y_;
            pose.pose.position.z = 0;
            path.poses.push_back(pose);
        }
        one_path_node = one_path_node->father_PathNode_;
    }

    pub_path_.publish(path);
    cout << "display the path." << endl;
}

void HybirdAStar::add2Openlist(PathNode* path_node) {
    if(beInList(path_node, open_list_) == -1) {
        open_list_.push_back(path_node);
    }
}

int main(int argc, char** argv) {
    cout << "begin the program." << endl;
    ros::init(argc, argv, "hybird_A_star");
    HybirdAStar test;
    return 0;
}