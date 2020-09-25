#include "opt_path.h"

OptPath::OptPath() {
    sub_ori_map_ = nh_.subscribe("/map", 2, &OptPath::subOriMap, this);
    sub_ori_path_ = nh_.subscribe("/own_path", 2, &OptPath::subOriPath, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/opt_path", 3);
    pub_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/poses", 3);
}

OptPath::~OptPath() {}

vector<geometry_msgs::Pose2D> OptPath::optThePath() {
    vector<geometry_msgs::Pose2D> ans;
    //sampleOptPath();
    pathPartition();

    for(int i = 0; i < tree_.size(); i++) {
        delete tree_[i];
    }
    tree_.clear();
    vector<vector<Node*> > nodes;
    nodes.resize(path_.size());
    Node* new_node;
    double dist = 0.1;

    double theta = atan2(path_[1].y - path_[0].y, path_[1].x - path_[0].x);
    new_node = new Node(path_[0].x, path_[0].y, theta);
    nodes[0].push_back(new_node);
    tree_.push_back(new_node);
    for(int j = 1; j < 6; j++) {
        new_node = new Node(path_[0].x, path_[0].y, theta + j * PI / 10);
        nodes[0].push_back(new_node);
        tree_.push_back(new_node);
        new_node = new Node(path_[0].x, path_[0].y, theta - j * PI / 10);
        nodes[0].push_back(new_node);
        tree_.push_back(new_node);
    }

    for(int i = 1; i < path_.size() - 1; i++) {
        theta = atan2(path_[i + 1].y - path_[i - 1].y, path_[i + 1].x - path_[i - 1].x);
        new_node = new Node(path_[i].x, path_[i].y, theta);
        nodes[i].push_back(new_node);
        tree_.push_back(new_node);
        for(int j = 1; j < 6; j++) {
            new_node = new Node(path_[i].x + dist * j * cos(theta + PI / 2), path_[i].y + dist * j * sin(theta + PI / 2), theta);
            nodes[i].push_back(new_node);
            tree_.push_back(new_node);
            new_node = new Node(path_[i].x - dist * j * cos(theta + PI / 2), path_[i].y - dist * j * sin(theta + PI / 2), theta);
            nodes[i].push_back(new_node);
            tree_.push_back(new_node);
        }
    }

    theta = atan2(path_[path_.size() - 1].y - path_[path_.size() - 2].y, path_[path_.size() - 1].x - path_[path_.size() - 2].x);
    new_node = new Node(path_[path_.size() - 1].x, path_[path_.size() - 1].y, theta);
    nodes[path_.size() - 1].push_back(new_node);
    tree_.push_back(new_node);
    for(int j = 1; j < 6; j++) {
        new_node = new Node(path_[path_.size() - 1].x, path_[path_.size() - 1].y, theta + j * PI / 10);
        nodes[path_.size() - 1].push_back(new_node);
        tree_.push_back(new_node);
        new_node = new Node(path_[path_.size() - 1].x, path_[path_.size() - 1].y, theta - j * PI / 10);
        nodes[path_.size() - 1].push_back(new_node);
        tree_.push_back(new_node);
    }

    geometry_msgs::PoseArray poses;
    poses.header.frame_id = "map";
    geometry_msgs::Pose one_pose;
    for(int i = 0; i < tree_.size(); i++) {
        one_pose.position.x = tree_[i]->x_;
        one_pose.position.y = tree_[i]->y_;
        one_pose.position.z = 0;
        one_pose.orientation.x = 0;
        one_pose.orientation.y = 0;
        one_pose.orientation.z = sin(tree_[i]->theta_ / 2);
        one_pose.orientation.w = cos(tree_[i]->theta_ / 2);
        poses.poses.push_back(one_pose);
    }
    pub_poses_.publish(poses);


    for(int i = 1; i < nodes.size(); i++) {
        for(int j = 0; j < nodes[i].size(); j++) {
            double min_dist = 99999;
            for(int k = 0; k < nodes[i - 1].size(); k++) {
                if(nodeObstacleCheck(nodes[i][j]->x_, nodes[i][j]->y_) != 1 && nodeObstacleCheck(nodes[i - 1][k]->x_, nodes[i][j]->y_) != 1) {
                    MyPath path = opt_path_.optSolveQuinticCurve2(nodes[i - 1][k]->x_, nodes[i - 1][k]->y_, nodes[i - 1][k]->theta_,
                                                                  nodes[i][j]->x_, nodes[i][j]->y_, nodes[i][j]->theta_);
                    if(pathObstacleCheck(path) != 1) {
                        if(min_dist > path.distance_ + nodes[i - 1][k]->distance_) {
                            min_dist = path.distance_ + nodes[i - 1][k]->distance_;
                            nodes[i][j]->x_list_.clear();
                            nodes[i][j]->y_list_.clear();
                            for(int n = 0; n < path.x_.size(); n++) {
                                nodes[i][j]->x_list_.push_back(path.x_[n]);
                                nodes[i][j]->y_list_.push_back(path.y_[n]);
                            }
                            nodes[i][j]->distance_ = min_dist;
                            nodes[i][j]->father_node_ = nodes[i - 1][k];
                        }
                    }
                }
            }
        }
    }


    

    return ans;
}

void OptPath::sampleOptPath() {
    for(int i = 0; i < 1000; i++) {
        int m = rand() % (path_.size() + 1);
        int n = rand() % (path_.size() + 1);
        int interim_num;
        if(m > n) {
            interim_num = n;
            n = m;
            m = interim_num;
        }
        if(n - m < 1) {
            break;
        }
        //cout << "test: " << m << ", " << n << endl;
        bool opt_successed_flag = true;
        double theta = atan2(path_[n].y - path_[m].y, path_[n].x - path_[m].x);
        for(int j = 0; j < hypot(path_[n].x - path_[m].x, path_[n].y - path_[m].y) / map_resolution_; j++) {
            double x = path_[m].x + j * map_resolution_ * cos(theta);
            double y = path_[m].y + j * map_resolution_ * sin(theta);
            if(nodeObstacleCheck(x, y) == true) {
                opt_successed_flag = false;
                break;
            }
        }
        if(opt_successed_flag == true) {
            path_.erase(path_.begin() + m + 1, path_.begin() + n);
        }
    }

    displayPath();

    // for(int i = 0; i < path_.size(); i++) {
    //     cout << "test: " << path_[i].x << ", " << path_[i].y << endl;
    // }
}

void OptPath::pathPartition() {
    double dist = 0;
    for(int i = 1; i < path_.size(); i++) {
        dist += hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
    }
    int partition_of_path;
    if(fabs(1.0 - dist / (int(dist) * 1.0)) > fabs(1.0 - dist / (int(dist) + 1.0))) {
        partition_of_path = int(dist) + 1;
    }
    else {
        partition_of_path = int(dist);
    }
    dist = dist / double(partition_of_path);
    double interim_dist = 0;
    vector<geometry_msgs::Pose2D> path;
    path.push_back(path_[0]);
    int n = 1;
    for(int i = 1; i < path_.size(); i++) {
        if(n >= partition_of_path) {
            break;
        }
        interim_dist += hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
        if(interim_dist >= n * dist) {
            path.push_back(path_[i]);
            n++;
        }
    }
    path.push_back(path_[path_.size() - 1]);

    path_.clear();
    for(int i = 0; i < path.size(); i++) {
        path_.push_back(path[i]);
    }
    displayPath();

    //cout << "test: " << dist << ", " << partition_of_path << endl;
}

void OptPath::subOriMap(nav_msgs::OccupancyGrid map) {
    ori_map_ = map;
    map_resolution_ = map.info.resolution;
}

void OptPath::subOriPath(nav_msgs::Path path) {
    ori_path_ = path;
    path_.clear();
    path_.resize(path.poses.size());
    for(int i = 0; i < path.poses.size(); i++) {
        path_[i].x = path.poses[i].pose.position.x;
        path_[i].y = path.poses[i].pose.position.y;
        path_[i].theta = 0;
    }
    //cout << "size of ori path: " << path_.size() << endl;

    optThePath();
}

void OptPath::displayPath() {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    for(int i = 0; i < path_.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = path_[i].x;
        pose.pose.position.y = path_[i].y;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        path.poses.push_back(pose);
    }

    pub_path_.publish(path);
}

void OptPath::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = (wx - ori_map_.info.origin.position.x) / ori_map_.info.resolution;
    my = (wy - ori_map_.info.origin.position.y) / ori_map_.info.resolution;
}

bool OptPath::nodeObstacleCheck(double x, double y) {
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

bool OptPath::pathObstacleCheck(MyPath path) {
    if(path.x_.size() == 0) {
        return true;
    }
    for(int i = 0; i < path.x_.size(); i++) {
        if(nodeObstacleCheck(path.x_[i], path.y_[i]) == 1) {
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    cout << "begin the opt path program." << endl;
    ros::init(argc, argv, "opt_path");
    OptPath test;
    ros::spin();
    return 0;
}