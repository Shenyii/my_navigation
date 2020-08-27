#include "generate_paths.h"

GeneratePaths::GeneratePaths() {
    cout << "generate some paths" << endl;
    min_r_ = 0.1;
    max_r_ = 10000;
    max_s_ = 5;
    pub_paths_ = nh_.advertise<sensor_msgs::PointCloud>("/paths", 3);
    obstacle_threshold_ = 65;
}

GeneratePaths::~GeneratePaths() {}

Path GeneratePaths::generatePaths(double start_x, double start_y, double goal_x, double goal_y, double resolution) {
    ref_x_ = start_x;
    ref_y_ = start_y;
    paths_.clear();
    double theta = atan2(goal_y - start_y, goal_x - start_x);
    ref_theta_ = theta;

    vector<Path> paths = curvePaths(hypot(goal_x - start_x, goal_y - start_y), 0, resolution);
    pathsToWorld(start_x, start_y, theta, paths);
    addThePaths(paths);

    displayThePath(paths);

    return bestPath();
}

Path GeneratePaths::generatePaths(double start_x, double start_y, double theta, double goal_x, double goal_y, double resolution) {
    ref_x_ = start_x;
    ref_y_ = start_y;
    ref_theta_ = theta;
    paths_.clear();
    Node new_node;
    vector<Node> path;
    double x, y;
    x = (goal_x - start_x) * cos(theta) + (goal_y - start_y) * sin(theta);    
    y = (goal_y - start_y) * cos(theta) - (goal_x - start_x) * sin(theta); 

    vector<Path> paths;
    paths = straightCurvePaths(x, y, resolution);
    pathsToWorld(start_x, start_y, theta, paths);
    addThePaths(paths);

    // paths.clear();
    // paths = curveStraightPaths(x, y, resolution);
    // pathsToWorld(start_x, start_y, theta, paths);
    // addThePaths(paths);

    displayThePath(paths);

    return bestPath();
}

void GeneratePaths::loadTheMap(nav_msgs::OccupancyGrid map) {
    ref_map_ = map;
}

vector<Path> GeneratePaths::curvePaths(double x, double y, double resolution) {
    vector<Path> paths;
    for(double theta = -PI / 2; theta <= PI / 2; theta += PI / 18) {
        Path one_path;
        Node one_node;
        if(fabs(theta) < 0.01) {
            for(double x0 = 0; x0 <= x; x0 += resolution) {
                one_node.x_ = x0;
                one_node.y_ = 0;
                if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                    one_path.path_.clear();
                    break;
                }
                one_path.path_.push_back(one_node);
            }
            one_path.length_ = x;
        }
        else if(theta > 0) {
            double c_r = fabs(x / 2 / sin(theta));
            if(c_r < min_r_) continue;
            for(double j = PI / 2 + theta; j >= PI / 2 - theta; j-= (resolution / c_r)) {
                one_node.x_ = c_r * cos(j) + x / 2;
                one_node.y_ = c_r * sin(j) - sqrt(c_r * c_r - x * x / 4);
                if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                    one_path.path_.clear();
                    break;
                }
                one_path.path_.push_back(one_node);
            }
            one_path.length_ = 2 * c_r * theta;
        }
        else {
            double c_r = fabs(x / 2 / sin(theta));
            if(c_r < min_r_) continue;
            for(double j = -PI / 2 + theta; j <= -PI / 2 - theta; j += (resolution / c_r)) {
                one_node.x_ = c_r * cos(j) + x / 2;
                one_node.y_ = c_r * sin(j) + sqrt(c_r * c_r - x * x / 4);
                if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                    one_path.path_.clear();
                    break;
                }
                one_path.path_.push_back(one_node);
            }
            one_path.length_ = 2 * c_r * fabs(theta);
        }
        if(one_path.path_.size() == 0) continue;
        paths.push_back(one_path);
    }

    return paths;
}

vector<Path> GeneratePaths::curvePaths2(double x, double y, double dist, double resolution) {
    vector<Path> paths;
    Path one_path;
    Node one_node;
    x = x - dist;
    double c_r = (x * x + y * y) / 2 / y;
    //cout << "dist: " << dist << ", r: " << c_r << endl;
    if(fabs(c_r) >= min_r_) {
        double theta = 2 * fabs(atan2(y, x));
        //cout << "r: " << c_r << ", theta: " << theta << ", dist: " << dist << endl;
        for(double j = 0; j < theta; j += (resolution / fabs(c_r))) {
            //cout << "test2: " << j << endl;
            one_node.x_ = fabs(c_r) * sin(j) + dist;
            one_node.y_ = c_r - c_r * cos(j);
            if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                one_path.path_.clear();
                break;
            }
            one_path.path_.push_back(one_node);
        }
        if(one_path.path_.size() == 0) return paths;
        one_path.length_ = fabs(c_r * theta);
        paths.push_back(one_path);
    }
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
            if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                one_path.path_.clear();
                break;
            }
            one_path.path_.push_back(one_node);
        }
        if(s != 0) {
            if(one_path.path_.size() == 0) continue;
        }
        part_of_path = curvePaths2(x, y, s, resolution);
        if(part_of_path.size() != 0) {
            for(int j = 0; j < part_of_path[0].path_.size(); j++) {
                one_node.x_ = part_of_path[0].path_[j].x_;
                one_node.y_ = part_of_path[0].path_[j].y_;
                if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                    one_path.path_.clear();
                    break;
                }
                one_path.path_.push_back(one_node);
            }
            if(one_path.path_.size() == 0) continue;
            one_path.length_ = s + part_of_path[0].length_;
            paths.push_back(one_path);
        }
    }

    return paths;
}

vector<Path> GeneratePaths::curveStraightPaths(double x, double y, double resolution) {
    vector<Path> paths;
    for(double angle = 0; angle < PI; angle += PI / 31) {
    //for(double angle = 0; angle < 0.001; angle += PI / 30) {
        Path one_path;
        Node one_node;
        double k = tan(angle);
        k = k == 0 ? 0.00001 : k;

        double r1 = (k * x - y + sqrt(pow(x * k - y, 2) + k * k * pow(x * k - y, 2))) / k / k;
        if(fabs(r1) < min_r_ || fabs(r1) > max_r_) {
            continue;
        }
        double x0 = (k * r1 - k * y + k * k * x) / (k * k + 1);
        double y0 = y + k * (x0 - x);
        double theta = atan2(x0, r1 - y0);
        theta = theta < 0 ? theta + 2 * PI : theta;
        if((r1 * sin(theta + 0.01) - r1 * sin(theta)) * (x - x0) < 0 ||
           (r1 * cos(theta) - r1 * cos(theta + 0.01)) * (y - y0) < 0) {
            continue;
        }
        for(double theta1 = 0; theta1 < theta; theta1 += resolution / r1) {
            one_node.x_ = r1 * sin(theta1);
            one_node.y_ = r1 - r1 * cos(theta1);
            if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                one_path.path_.clear();
                break;
            }
            one_path.path_.push_back(one_node);
        }
        if(one_path.path_.size() == 0) continue;
        int n = hypot(x - x0, y - y0) / resolution;
        for(int i = 0; i <= n; i++) {
            one_node.x_ = x0 + (x - x0) * i / n;
            one_node.y_ = y0 + (y - y0) * i / n;
            if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                one_path.path_.clear();
                break;
            }
            one_path.path_.push_back(one_node);
        }
        if(one_path.path_.size() == 0) continue;
        one_path.length_ = r1 * theta + hypot(x - x0, y - y0);
        paths.push_back(one_path);
    }

    for(double angle = 0; angle < PI; angle += PI / 31) {
        Path one_path;
        Node one_node;
        double k = tan(angle);
        k = k == 0 ? 0.00001 : k;
        one_path.path_.clear();
        double r1 = (k * x - y - sqrt(pow(x * k - y, 2) + k * k * pow(x * k - y, 2))) / k / k;
        if(fabs(r1) < min_r_ || fabs(r1) > max_r_) {
            continue;
        }
        double x0 = (k * r1 - k * y + k * k * x) / (k * k + 1);
        double y0 = y + k * (x0 - x);
        double theta = atan2(x0, y0 - r1);
        theta = theta < 0 ? theta + 2 * PI : theta;
        if((r1 * sin(theta) - r1 * sin(theta + 0.01)) * (x - x0) < 0 ||
           (r1 * cos(theta) - r1 * cos(theta + 0.01)) * (y - y0) < 0) {
            continue;
        }
        for(double theta1 = 0; theta1 < theta; theta1 += resolution / -r1) {
            one_node.x_ = -r1 * sin(theta1);
            one_node.y_ = r1 - r1 * cos(theta1);
            if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                one_path.path_.clear();
                break;
            }
            one_path.path_.push_back(one_node);
        }
        if(one_path.path_.size() == 0) continue;
        int n = hypot(x - x0, y - y0) / resolution;
        for(int i = 0; i <= n; i++) {
            one_node.x_ = x0 + (x - x0) * i / n;
            one_node.y_ = y0 + (y - y0) * i / n;
            if(obstacleCheck(ref_x_, ref_y_, ref_theta_, one_node.x_, one_node.y_)) {
                one_path.path_.clear();
                break;
            }
            one_path.path_.push_back(one_node);
        }
        if(one_path.path_.size() == 0) continue;
        one_path.length_ = fabs(r1 * theta) + hypot(x - x0, y - y0);
        paths.push_back(one_path);
    }

    return paths;
}

void GeneratePaths::pathsToWorld(double x, double y, double theta, vector<Path>& paths) {
    double c = cos(theta);
    double s = sin(theta);
    for(int i = 0; i < paths.size(); i++) {
        for(int j = 0; j < paths[i].path_.size(); j++) {
            double x0 = paths[i].path_[j].x_ * c - paths[i].path_[j].y_ * s + x;
            double y0 = paths[i].path_[j].x_ * s + paths[i].path_[j].y_ * c + y;
            paths[i].path_[j].x_ = x0;
            paths[i].path_[j].y_ = y0;
        }
    }
}

void GeneratePaths::addThePaths(vector<Path> paths) {
    for(int i = 0; i < paths.size(); i++) {
        paths_.push_back(paths[i]);
    }
}

bool GeneratePaths::obstacleCheck(double ref_x, double ref_y, double ref_theta, double x, double y) {
    int mx;
    int my;
    double x0;
    double y0;
    double c = cos(ref_theta);
    double s = sin(ref_theta);
    //cout << ref_x << ", " << ref_y << ", " << ref_theta << ", " << ref_map_.data.size() << endl;
    x0 = x * c - y * s + ref_x;
    y0 = x * s + y * c + ref_y;
    mx = (x0 - ref_map_.info.origin.position.x) / ref_map_.info.resolution;
    my = (y0 - ref_map_.info.origin.position.y) / ref_map_.info.resolution;
    //cout << ref_map_.data.size() << ", " << mx + my * ref_map_.info.width << ", " << x0 << ", " << y0 << endl;
    if(mx + my * ref_map_.info.width > ref_map_.data.size() || mx + my * ref_map_.info.width < 0) {
        return false;
    }
    int obstacle_state = ref_map_.data[mx + my * ref_map_.info.width];
    if(obstacle_state < obstacle_threshold_ && obstacle_state != -1) {
        return false;
    }
    else {
        return true;
    }
}

Path GeneratePaths::bestPath() {
    double min_dist = 999999;
    int index = -1;
    for(int i = 0; i < paths_.size(); i++) {
        if(paths_[i].length_ < min_dist) {
            min_dist = paths_[i].length_;
            index = i;
        }
    }
    if(index == -1) {
        Path empty_path;
        return empty_path;
    }
    else {
        return paths_[index];
    }
}

void GeneratePaths::displayThePath(vector<Path> paths) {
    sensor_msgs::PointCloud points;
    points.header.frame_id = "map";
    geometry_msgs::Point32 point;
    for(int i = 0; i < paths.size(); i++) {
        cout << paths[i].length_ << endl;
        for(int j = 0; j < paths[i].path_.size(); j++) {
            point.x = paths[i].path_[j].x_;
            point.y = paths[i].path_[j].y_;
            point.z = 0.02;
            points.points.push_back(point);
        }
    }

    pub_paths_.publish(points);
}