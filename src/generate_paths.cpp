#include "generate_paths.h"

GeneratePaths::GeneratePaths() {
    cout << "generate some paths" << endl;
}

GeneratePaths::~GeneratePaths() {}

vector<Path> GeneratePaths::generatePaths(double start_x, double start_y, double goal_x, double goal_y, double resolution) {
    paths_.clear();
    double x, y, theta;
    theta = atan2(goal_y - start_y, goal_x - start_x);
    x = (goal_x - start_x) * cos(theta) + (goal_y - start_y) * sin(theta);
    y = (goal_y - start_y) * cos(theta) - (goal_x - start_x) * sin(theta);
    vector<Path> paths = curvePaths(x, 0, resolution);
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
}

vector<Path> GeneratePaths::straightCurvePaths(double x, double y, double resolution) {}

vector<Path> GeneratePaths::curveStraightPaths(double x, double y, double resolution) {}

void GeneratePaths::pathsToWorld(double x, double y, double theta, vector<Path>& paths) {}