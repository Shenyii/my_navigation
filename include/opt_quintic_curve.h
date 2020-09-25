#ifndef OPT_QUINTIC_CURVE_H
#define OPT_QUINTIC_CURVE_H

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include "osqp.h"

#define PI 3.14159265

using namespace std;
using namespace Eigen;

class MyPath {
public:
    vector<double> x_;
    vector<double> y_;
    vector<double> theta_;
    double distance_;

    MyPath() {
        distance_ = 0;
    }
};

class OptQuinticCurve {
public:
    OptQuinticCurve();
    ~OptQuinticCurve();

    void optSolveQuinticCurve(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta);
    MyPath optSolveQuinticCurve2(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta);

private:
    double ori_start_x_;
    double ori_start_y_;
    double ori_start_theta_;
    double ori_goal_x_;
    double ori_goal_y_;
    double ori_goal_theta_;
    double start_x_;
    double start_y_;
    double start_theta_;
    double goal_x_;
    double goal_y_;
    double goal_theta_;
    double tf_;
    double min_v_;
    double max_acc_;
    int acc_constraint_num_;

    ros::NodeHandle nh_;
    sensor_msgs::PointCloud points_;
    ros::Publisher pub_points_;

    Matrix<double, Dynamic, 1> solveProblem(Matrix<double, Dynamic, Dynamic> H, Matrix<double, Dynamic, Dynamic> g,
                                            Matrix<double, Dynamic, Dynamic> A,Matrix<double, Dynamic, Dynamic> lb, Matrix<double, Dynamic, Dynamic> ub);
    
    void matrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M);
    
    int upperMatrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M);
};

#endif