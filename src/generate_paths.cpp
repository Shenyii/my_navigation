#include "generate_paths.h"

GeneratePaths::GeneratePaths() {
    cout << "generate some paths" << endl;
    c_r_.push_back(1);
    c_r_.push_back(2);
    c_r_.push_back(4);
    c_r_.push_back(8);
    c_r_.push_back(16);
}

GeneratePaths::~GeneratePaths() {}

vector<Path> GeneratePaths::generatePaths(double start_x, double start_y, double goal_x, double goal_y, double resolution) {
    paths_.clear();
    double x, y, theta;
    theta = atan2(goal_y - start_y, goal_x - start_x);
    x = (goal_x - start_x) * cos(theta) + (goal_y - start_y) * sin(theta);
    y = (goal_y - start_y) * cos(theta) - (goal_x - start_x) * sin(theta);
    vector<Path> paths = curvePaths(x, y, resolution);
    pathsToWorld(start_x, start_y, theta, paths);
}

vector<Path> GeneratePaths::generatePaths(double start_x, double start_y, double theta, double goal_x, double goal_y, double resolution) {
    paths_.clear();
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
                double x0 = 0;
                double y0 = 0;
                double theta0 = theta;
                for(double j = 0; j < 2 * theta; j+=(resolution / c_r_[i])) {
                    theta0 -= j;
                    x0 += resolution * cos(theta0);
                    y0 += resolution * sin(theta0);
                    one_node.x_ = x0;
                    one_node.y_ = y0;
                    one_path.path_.push_back(one_node);
                }
            }
        }
    }
    else {}
}

vector<Path> GeneratePaths::straightCurvePaths(double x, double y, double resolution) {}

vector<Path> GeneratePaths::curveStraightPaths(double x, double y, double resolution) {}

void GeneratePaths::pathsToWorld(double x, double y, double theta, vector<Path>& paths) {}