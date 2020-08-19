#include "generate_paths.h"

GeneratePaths::GeneratePaths() {
    cout << "generate some paths" << endl;
    c_r_.push_back(1);
    c_r_.push_back(2);
    c_r_.push_back(4);
    c_r_.push_back(8);
    c_r_.push_back(16);
    pub_paths_ = nh_.advertise<sensor_msgs::PointCloud>("/paths", 3);
}

GeneratePaths::~GeneratePaths() {}

vector<Path> GeneratePaths::generatePaths(double start_x, double start_y, double goal_x, double goal_y, double resolution) {
    paths_.clear();
    double theta = atan2(goal_y - start_y, goal_x - start_x);
    vector<Path> paths = curvePaths(hypot(goal_x - start_x, goal_y - start_y), 0, resolution);
    pathsToWorld(start_x, start_y, theta, paths);
    
    addThePaths(paths);

    displayThePath(paths);

    return paths_;
}

vector<Path> GeneratePaths::generatePaths(double start_x, double start_y, double theta, double goal_x, double goal_y, double resolution) {
    paths_.clear();
    Node new_node;
    vector<Node> path;
    double x, y;
    x = (goal_x - start_x) * cos(theta) + (goal_y - start_y) * sin(theta);
    y = (goal_y - start_y) * cos(theta) - (goal_x - start_x) * sin(theta);
    vector<Path> paths = straightCurvePaths(x, y, resolution);
    pathsToWorld(start_x, start_y, theta, paths);
    /////////add the path

    paths.clear();
    paths = curveStraightPaths(x, y, resolution);
    pathsToWorld(start_x, start_y, theta, paths);
    /////////add the path
}

vector<Path> GeneratePaths::curvePaths(double x, double y, double resolution) {
    vector<Path> paths;
    if(y == 0) {
        for(int i = 0; i < c_r_.size(); i++) {
            Path one_path;
            Node one_node;
            if(x <= 2 * c_r_[i]) {
                double theta = asin(x / 2 / c_r_[i]);
                one_path.path_.clear();
                for(double j = PI / 2 + theta; j >= PI / 2 - theta; j-=(resolution / c_r_[i])) {
                    one_node.x_ = c_r_[i] * cos(j) + x / 2;
                    one_node.y_ = c_r_[i] * sin(j) - sqrt(c_r_[i] * c_r_[i] - x * x / 4);
                    one_path.path_.push_back(one_node);
                }
                paths.push_back(one_path);
                theta = -theta;
                one_path.path_.clear();
                for(double j = -PI / 2 + theta; j <= -PI / 2 - theta; j+=(resolution / c_r_[i])) {
                    one_node.x_ = c_r_[i] * cos(j) + x / 2;
                    one_node.y_ = c_r_[i] * sin(j) + sqrt(c_r_[i] * c_r_[i] - x * x / 4);
                    one_path.path_.push_back(one_node);
                }
                paths.push_back(one_path);
            }
        }
    }
    else {
        Path one_path;
        Node one_node;
        double c_r = hypot(x, y) / 2 / y;
        if(fabs(c_r) >= c_r_[0]) {
            double theta = 2 * atan2(y, x);
            double x0 = 0;
            double y0 = 0;
            double theta0 = 0;
            for(double j = 0; j < theta; j+=(resolution / fabs(c_r))) {
                theta0 += theta / fabs(theta) * j;
                x0 += resolution * cos(theta0);
                y0 += resolution * sin(theta0);
                one_node.x_ = x0;
                one_node.y_ = y0;
                one_path.path_.push_back(one_node);
            }
            paths.push_back(one_path);
        }
    }

    return paths;
}

vector<Path> GeneratePaths::straightCurvePaths(double x, double y, double resolution) {}

vector<Path> GeneratePaths::curveStraightPaths(double x, double y, double resolution) {}

void GeneratePaths::pathsToWorld(double x, double y, double theta, vector<Path>& paths) {
    double c = cos(theta);
    double s = sin(theta);
    cout << "size of paths is: " << paths.size() << endl;
    for(int i = 0; i < paths.size(); i++) {
        for(int j = 0; j < paths[i].path_.size(); j++) {
            double x0 = paths[i].path_[j].x_ * c - paths[i].path_[j].y_ * s;
            double y0 = paths[i].path_[j].x_ * s + paths[i].path_[j].y_ * c;
            paths[i].path_[j].x_ = x0;
            paths[i].path_[j].y_ = y0;
        }
    }
    cout << "testtt0" << endl;
}

void GeneratePaths::addThePaths(vector<Path> paths) {
    for(int i = 0; i < paths.size(); i++) {
        paths_.push_back(paths[i]);
    }
}

void GeneratePaths::displayThePath(vector<Path> paths) {
    sensor_msgs::PointCloud points;
    points.header.frame_id = "map";
    geometry_msgs::Point32 point;
    for(int i = 0; i < paths.size(); i++) {
        for(int j = 0; j < paths[i].path_.size(); j++) {
            point.x = paths[i].path_[j].x_;
            point.y = paths[i].path_[j].y_;
            point.z = 0.02;
            points.points.push_back(point);
        }
    }

    pub_paths_.publish(points);
}