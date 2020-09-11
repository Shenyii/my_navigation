#ifndef OPT_BEZIER_CURVE_H
#define OPT_BEZIER_CURVE_H

#include <iostream>

#include <ros/ros.h>
#include <Eigen/Dense>
#include "osqp.h"

using namespace std;
using namespace Eigen;

class OptBezierCurve {
public:
    OptBezierCurve();
    ~OptBezierCurve();

    bool getOptBezierCurve(double start_x, double start_y, double end_x, double end_y);

    bool setBezierN(int n);

private:
    int bezier_n_;
    double min_v_;
    double max_v_;
    double min_acc_;
    double max_acc_;

    void setMatrixHg(Matrix<double, Dynamic, Dynamic>& H, Matrix<double, Dynamic, Dynamic>& g);
};

#endif