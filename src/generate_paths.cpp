#include "generate_paths.h"

GeneratePaths::GeneratePaths() {
    cout << "generate some paths" << endl;
    min_r_ = 1;
    max_s_ = 5;
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

    vector<Path> paths;
    // paths = straightCurvePaths(x, y, resolution);
    // pathsToWorld(start_x, start_y, theta, paths);
    // addThePaths(paths);

    paths.clear();
    paths = curveStraightPaths(x, y, resolution);
    pathsToWorld(start_x, start_y, theta, paths);
    addThePaths(paths);

    displayThePath(paths);

    return paths_;
}

vector<Path> GeneratePaths::curvePaths(double x, double y, double resolution) {
    vector<Path> paths;
    if(y == 0) {
        for(double theta = -PI / 2; theta <= PI / 2; theta += PI / 18) {
            Path one_path;
            Node one_node;
            if(fabs(theta) < 0.01) {
                for(double x0 = 0; x0 <= x; x0 += resolution) {
                    one_node.x_ = x0;
                    one_node.y_ = 0;
                    one_path.path_.push_back(one_node);
                }
            }
            else if(theta > 0) {
                double c_r = fabs(x / 2 / sin(theta));
                if(c_r < min_r_) continue;
                for(double j = PI / 2 + theta; j >= PI / 2 - theta; j-= (resolution / c_r)) {
                    one_node.x_ = c_r * cos(j) + x / 2;
                    one_node.y_ = c_r * sin(j) - sqrt(c_r * c_r - x * x / 4);
                    one_path.path_.push_back(one_node);
                }
            }
            else {
                double c_r = fabs(x / 2 / sin(theta));
                if(c_r < min_r_) continue;
                for(double j = -PI / 2 + theta; j <= -PI / 2 - theta; j += (resolution / c_r)) {
                    one_node.x_ = c_r * cos(j) + x / 2;
                    one_node.y_ = c_r * sin(j) + sqrt(c_r * c_r - x * x / 4);
                    one_path.path_.push_back(one_node);
                }
            }
            paths.push_back(one_path);
        }
    }
    else {
        Path one_path;
        Node one_node;
        double c_r = (x * x + y * y) / 2 / y;
        if(fabs(c_r) >= min_r_) {
            double theta = 2 * fabs(atan2(y, x));
            for(double j = 0; j < theta; j += (resolution / fabs(c_r))) {
                one_node.x_ = fabs(c_r) * sin(j);
                one_node.y_ = c_r - c_r * cos(j);
                one_path.path_.push_back(one_node);
            }
            paths.push_back(one_path);
        }
    }

    return paths;
}

vector<Path> GeneratePaths::straightCurvePaths(double x, double y, double resolution) {
    vector<Path> paths;
    for(double s = 0; s < max_s_; s += 0.5) {
        vector<Path> part_of_path;
        Path one_path;
        Node one_node;
        for(double x0 = 0; x0 < s; x0 += resolution) {
            one_node.x_ = x0;
            one_node.y_ = 0;
            one_path.path_.push_back(one_node);
        }
        part_of_path = curvePaths(x - s, y, resolution);
        for(int i = 0; i < part_of_path.size(); i++) {
            for(int j = 0; j < part_of_path[i].path_.size(); j++) {
                one_node.x_ = part_of_path[i].path_[j].x_ + s;
                one_node.y_ = part_of_path[i].path_[j].y_;
                one_path.path_.push_back(one_node);
            }
        }
        if(part_of_path.size() != 0) {
            paths.push_back(one_path);
        }
    }

    return paths;
}

vector<Path> GeneratePaths::curveStraightPaths(double x, double y, double resolution) {
    vector<Path> paths;
    // for(double angle = -PI; angle < PI; angle += PI / 18) {
    //     double k = tan(angle);
    //     if(x >= 0) {
    //         if(x - y / k <= 0) continue;
    //         if(hypot(y / k, y) < x - y / k) continue;
    //     }
    //     else {}
    // }
    double k = (y + 5) / (x + 5);
    double r1 = (5 - 5 * k + sqrt(pow(5 * k - 5, 2) + k * k * pow(5 * k - 5, 2))) / k / k;
    double r2 = (5 - 5 * k - sqrt(pow(5 * k - 5, 2) + k * k * pow(5 * k - 5, 2))) / k / k;
    cout << "r1, r2: " << r1 << ", " << r2 << endl;
    Path one_path;
    Node one_node;
    for(double x = -100; x < 100; x += 0.1) {
        one_node.x_ = x;
        one_node.y_ = k * x + 5 * k - 5;
        one_path.path_.push_back(one_node);
    }
    for(double theta = -PI; theta < PI; theta += PI / 180) {
        one_node.x_ = r1 * cos(theta);
        one_node.y_ = r1 * sin(theta) + r1;
        one_path.path_.push_back(one_node);
        one_node.x_ = r2 * cos(theta);
        one_node.y_ = r2 * sin(theta) + r2;
        one_path.path_.push_back(one_node);
    }
    paths.push_back(one_path);

    return paths;
}

void GeneratePaths::pathsToWorld(double x, double y, double theta, vector<Path>& paths) {
    double c = cos(theta);
    double s = sin(theta);
    cout << "size of paths is: " << paths.size() << endl;
    for(int i = 0; i < paths.size(); i++) {
        for(int j = 0; j < paths[i].path_.size(); j++) {
            double x0 = paths[i].path_[j].x_ * c - paths[i].path_[j].y_ * s + x;
            double y0 = paths[i].path_[j].x_ * s + paths[i].path_[j].y_ * c + y;
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