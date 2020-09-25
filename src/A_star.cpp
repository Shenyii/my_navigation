#include "A_star.h"

AStar::AStar()
: start_x_(0), start_y_(0), find_path_flag_(0) {
    obstacle_threshold_ = 65;
    find_path_flag_ = 0;
    sub_ori_map_ = nh_.subscribe("/map", 2, &AStar::subOriMap, this);
    sub_start_pose_ = nh_.subscribe("/initialpose", 2, &AStar::subStartPose, this);
    sub_goal_pose_ = nh_.subscribe("/move_base_simple/goal", 2, &AStar::subGoalPose, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/own_path", 3);
    pub_points = nh_.advertise<sensor_msgs::PointCloud>("/tree", 3);
}

AStar::~AStar() {}

void AStar::subOriMap(nav_msgs::OccupancyGrid map) {
    ori_map_ = map;
    extend_dist_ = 2 * map.info.resolution;
    cout << "get the origin map." << extend_dist_ << endl;
}

void AStar::subStartPose(geometry_msgs::PoseWithCovarianceStamped start_pose) {
    start_x_ = start_pose.pose.pose.position.x;
    start_y_ = start_pose.pose.pose.position.y;
}

void AStar::subGoalPose(geometry_msgs::PoseStamped goal_pose) {
    goal_x_ = goal_pose.pose.position.x;
    goal_y_ = goal_pose.pose.position.y;
    find_path_flag_ = 0;
    searchThePath();
}

void AStar::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = (wx - ori_map_.info.origin.position.x) / ori_map_.info.resolution;
    my = (wy - ori_map_.info.origin.position.y) / ori_map_.info.resolution;
}

bool AStar::searchThePath() {
    for(int i = 0; i < close_list_.size(); i++) {
        delete close_list_[i];
    }
    close_list_.clear();
    open_list_.clear();
    Node* new_node = new Node(start_x_, start_y_, 0, 0, NULL);
    close_list_.push_back(new_node);
    open_list_.push_back(new_node);
    while(beInList(goal_x_, goal_y_, close_list_) == -1 && find_path_flag_ == 0 && open_list_.size() > 0 && ros::ok()) {
        extendCloseList();
        //displayCloseList();
        //getchar();
    }
    if(find_path_flag_ == 1) {
        getThePath();
        cout << "find the path" << endl;
    }
    else {
        cout << "can't find the path." << endl;
    }
}

bool AStar::nodeObstacleCheck(double x, double y) {
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

void AStar::extendCloseList() {
    //cout << "extend close list" << endl;
    sort(open_list_.begin(), open_list_.end(), nodeLess);
    Node* father_node = open_list_[0];
    open_list_.erase(open_list_.begin(), open_list_.begin() + 1);
    int index;

    if(nodeObstacleCheck(father_node->x_ + extend_dist_, father_node->y_) == false) {
        index = beInList(father_node->x_ + extend_dist_, father_node->y_, close_list_);
        if(index == -1) {
            double dist = father_node->distance_ + extend_dist_;
            //double value = dist + fabs(father_node->x_ + extend_dist_ - goal_x_) + fabs(father_node->y_ - goal_y_);
            double value = dist + hypot(father_node->x_ + extend_dist_ - goal_x_, father_node->y_ - goal_y_);
            //double value = hypot(father_node->x_ + extend_dist_ - start_x_, father_node->y_ - start_y_) + hypot(father_node->x_ + extend_dist_ - goal_x_, father_node->y_ - goal_y_);
            Node* new_node = new Node(father_node->x_ + extend_dist_, father_node->y_, dist, value, father_node);
            close_list_.push_back(new_node);
            open_list_.push_back(new_node);
            if(findPathCheck() == 1) {
                return;
            }
        }
        else {
            double dist = father_node->distance_ + extend_dist_;
            if(dist < close_list_[index]->distance_) {
                close_list_[index]->distance_ = dist;
                //close_list_[index]->heuristics_value_ = dist + fabs(father_node->x_ + extend_dist_ - goal_x_) + fabs(father_node->y_ - goal_y_);
                close_list_[index]->heuristics_value_ = dist + hypot(father_node->x_ + extend_dist_ - goal_x_, father_node->y_ - goal_y_);
                //close_list_[index]->heuristics_value_ = hypot(father_node->x_ + extend_dist_ - start_x_, father_node->y_ - start_y_) + hypot(father_node->x_ + extend_dist_ - goal_x_, father_node->y_ - goal_y_);
                close_list_[index]->father_node_ = father_node;
            }
        }
    }

    if(nodeObstacleCheck(father_node->x_ - extend_dist_, father_node->y_) == false) {
        index = beInList(father_node->x_ - extend_dist_, father_node->y_, close_list_);
        if(index == -1) {
            double dist = father_node->distance_ + extend_dist_;
            //double value = dist + fabs(father_node->x_ - extend_dist_ - goal_x_) + fabs(father_node->y_ - goal_y_);
            double value = dist + hypot(father_node->x_ - extend_dist_ - goal_x_, father_node->y_ - goal_y_);
            //double value = hypot(father_node->x_ - extend_dist_ - start_x_, father_node->y_ - start_y_) + hypot(father_node->x_ - extend_dist_ - goal_x_, father_node->y_ - goal_y_);
            Node* new_node = new Node(father_node->x_ - extend_dist_, father_node->y_, dist, value, father_node);
            close_list_.push_back(new_node);
            open_list_.push_back(new_node);
            if(findPathCheck() == 1) {
                return;
            }
        }
        else {
            double dist = father_node->distance_ + extend_dist_;
            if(dist < close_list_[index]->distance_) {
                close_list_[index]->distance_ = dist;
                //close_list_[index]->heuristics_value_ = dist + fabs(father_node->x_ - extend_dist_ - goal_x_) + fabs(father_node->y_ - goal_y_);
                close_list_[index]->heuristics_value_ = dist + hypot(father_node->x_ - extend_dist_ - goal_x_, father_node->y_ - goal_y_);
                //close_list_[index]->heuristics_value_ = hypot(father_node->x_ - extend_dist_ - start_x_, father_node->y_ - start_y_) + hypot(father_node->x_ - extend_dist_ - goal_x_, father_node->y_ - goal_y_);
                close_list_[index]->father_node_ = father_node;
            }
        }
    }

    if(nodeObstacleCheck(father_node->x_, father_node->y_ + extend_dist_) == false) {
        index = beInList(father_node->x_, father_node->y_ + extend_dist_, close_list_);
        if(index == -1) {
            double dist = father_node->distance_ + extend_dist_;
            //double value = dist + fabs(father_node->x_ - goal_x_) + fabs(father_node->y_ + extend_dist_ - goal_y_);
            double value = dist + hypot(father_node->x_ - goal_x_, father_node->y_ + extend_dist_ - goal_y_);
            //double value = hypot(father_node->x_ - start_x_, father_node->y_ + extend_dist_ - start_y_) + hypot(father_node->x_ - goal_x_, father_node->y_ + extend_dist_ - goal_y_);
            Node* new_node = new Node(father_node->x_, father_node->y_ + extend_dist_, dist, value, father_node);
            close_list_.push_back(new_node);
            open_list_.push_back(new_node);
            if(findPathCheck() == 1) {
                return;
            }
        }
        else {
            double dist = father_node->distance_ + extend_dist_;
            if(dist < close_list_[index]->distance_) {
                close_list_[index]->distance_ = dist;
                //close_list_[index]->heuristics_value_ = dist + fabs(father_node->x_ - goal_x_) + fabs(father_node->y_ + extend_dist_ - goal_y_);
                close_list_[index]->heuristics_value_ = dist + hypot(father_node->x_ - goal_x_, father_node->y_ + extend_dist_ - goal_y_);
                //close_list_[index]->heuristics_value_ = hypot(father_node->x_ - start_x_, father_node->y_ + extend_dist_ - start_y_) + hypot(father_node->x_ - goal_x_, father_node->y_ + extend_dist_ - goal_y_);
                close_list_[index]->father_node_ = father_node;
            }
        }
    }

    if(nodeObstacleCheck(father_node->x_, father_node->y_ - extend_dist_) == false) {
        index = beInList(father_node->x_, father_node->y_ - extend_dist_, close_list_);
        if(index == -1) {
            double dist = father_node->distance_ + extend_dist_;
            //double value = dist + fabs(father_node->x_ - goal_x_) + fabs(father_node->y_ - extend_dist_ - goal_y_);
            double value = dist + hypot(father_node->x_ - goal_x_, father_node->y_ - extend_dist_ - goal_y_);
            //double value = hypot(father_node->x_ - start_x_, father_node->y_ - extend_dist_ - start_y_) + hypot(father_node->x_ - goal_x_, father_node->y_ - extend_dist_ - goal_y_);
            Node* new_node = new Node(father_node->x_, father_node->y_ - extend_dist_, dist, value, father_node);
            close_list_.push_back(new_node);
            open_list_.push_back(new_node);
            if(findPathCheck() == 1) {
                return;
            }
        }
        else {
            double dist = father_node->distance_ + extend_dist_;
            if(dist < close_list_[index]->distance_) {
                close_list_[index]->distance_ = dist;
                //close_list_[index]->heuristics_value_ = dist + fabs(father_node->x_ - goal_x_) + fabs(father_node->y_ - extend_dist_ - goal_y_);
                close_list_[index]->heuristics_value_ = dist + hypot(father_node->x_ - goal_x_, father_node->y_ - extend_dist_ - goal_y_);
                //close_list_[index]->heuristics_value_ = hypot(father_node->x_ - start_x_, father_node->y_ - extend_dist_ - start_y_) + hypot(father_node->x_ - goal_x_, father_node->y_ - extend_dist_ - goal_y_);
                close_list_[index]->father_node_ = father_node;
            }
        }
    }
}

int AStar::beInList(Node* node, vector<Node*> list) {
    for(int i = 0; i < list.size(); i++) {
        int x0;
        int y0;
        int x1;
        int y1;
        worldToMap(list[i]->x_, list[i]->y_, x0, y0);
        worldToMap(node->x_, node->y_, x1, y1);
        if((x0 == x1) && (y0 == y1)) {
            return i;
        }
    }
    return -1;
}

int AStar::beInList(double x, double y, vector<Node*> list) {
    for(int i = 0; i < list.size(); i++) {
        int x0;
        int y0;
        int x1;
        int y1;
        worldToMap(list[i]->x_, list[i]->y_, x0, y0);
        worldToMap(x, y, x1, y1);
        if((x0 == x1) && (y0 == y1)) {
            return i;
        }
    }
    return -1;
}

int AStar::findPathCheck() {
    double x = close_list_[close_list_.size() - 1]->x_;
    double y = close_list_[close_list_.size() - 1]->y_;
    if(fabs(x - goal_x_) < extend_dist_ && fabs(y - goal_y_) < extend_dist_) {
        find_path_flag_ = 1;
        return 1;
    }
    else {
        find_path_flag_ = 0;
        return 0;
    }
}

void AStar::getThePath() {
    Node* node = close_list_[close_list_.size() - 1];
    vector<geometry_msgs::Point32> path;
    geometry_msgs::Point32 point;
    point.x = goal_x_;
    point.y = goal_y_;
    point.z = 0;
    path.push_back(point);
    while(node != NULL) {
        point.x = node->x_;
        point.y = node->y_;
        point.z = 0;
        path.push_back(point);
        node = node->father_node_;
    }
    reverse(path.begin(), path.end());
    path_.header.frame_id = "map";
    path_.poses.clear();
    for(int i = 0; i < path.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = path[i].x;
        pose.pose.position.y = path[i].y;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        path_.poses.push_back(pose);
    }

    pub_path_.publish(path_);
}

void AStar::displayCloseList() {
    sensor_msgs::PointCloud points;
    points.points.resize(close_list_.size());
    points.header.frame_id = "map";
    for(int i = 0; i < close_list_.size(); i++) {
        points.points[i].x = close_list_[i]->x_;
        points.points[i].y = close_list_[i]->y_;
        points.points[i].z = 0;
    }
    pub_points.publish(points);

    // cout << "the size of open list is: " << open_list_.size() << endl;
    // for(int i = 0; i < open_list_.size(); i++) {
    //     cout << open_list_[i]->x_ << ", " << open_list_[i]->y_ << ", " << open_list_[i]->distance_ << ", " << open_list_[i]->heuristics_value_ << endl;
    // }
}

void AStar::add2Openlist(Node* node) {}

int main(int argc, char** argv) {
    cout << "begin the A_star program." << endl;
    ros::init(argc, argv, "A_star");
    AStar test;
    ros::spin();
    return 0;
}