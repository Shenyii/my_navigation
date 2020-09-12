#ifndef OPT_BEZIER_CURVE_H
#define OPT_BEZIER_CURVE_H

#include <iostream>

#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include <Eigen/Dense>
#include "osqp.h"

#define PI 3.14159265

using namespace std;
using namespace Eigen;

class OptBezierCurve {
public:
    OptBezierCurve();
    ~OptBezierCurve();

    bool getOptBezierCurve(double start_x, double start_y, double start_theta, double end_x, double end_y, double end_theta);

    bool setBezierN(int n);

private:
    int bezier_n_;
    double min_v_;
    double max_v_;
    double min_acc_;
    double max_acc_;

    void setMatrixHg(Matrix<double, Dynamic, Dynamic>& H, Matrix<double, Dynamic, Dynamic>& g);
    void setContraint(double start_x, double start_y, double start_theta, double end_x, double end_y, double end_theta,
                      Matrix<double, Dynamic, Dynamic>& A, Matrix<double, Dynamic, Dynamic>& lb, Matrix<double, Dynamic, Dynamic>& ub);

    double integralSpecialFun(int a, int b);
    double C_n_m(int n, int m);

    Matrix<double, Dynamic, 1> solveProblem(Matrix<double, Dynamic, Dynamic> H, Matrix<double, Dynamic, Dynamic> g,
                                      Matrix<double, Dynamic, Dynamic> A,Matrix<double, Dynamic, Dynamic> lb, Matrix<double, Dynamic, Dynamic> ub);
    
    void matrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M);
    
    int upperMatrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M);

    ros::NodeHandle nh_;
    sensor_msgs::PointCloud points_;
    ros::Publisher pub_points_;
};

#endif