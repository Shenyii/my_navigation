#include "opt_bezier_curve.h"

OptBezierCurve::OptBezierCurve() {
    bezier_n_ = 6;
    min_v_ = -1;
    max_v_ = 1;
    min_acc_ = -0.5;
    max_acc_ = 0.5;
}

OptBezierCurve::~OptBezierCurve() {}

bool OptBezierCurve::getOptBezierCurve(double start_x, double start_y, double end_x, double end_y) {
    Matrix<double, Dynamic, Dynamic> H, g, A, lb, ub;
    setMatrixHg(H, g);
    
    A.resize(4 * bezier_n_ - 6, 2 * bezier_n_);
    lb.resize(4 * bezier_n_ - 4, 1);
    ub.resize(4 * bezier_n_ - 4, 1);
}

void OptBezierCurve::setMatrixHg(Matrix<double, Dynamic, Dynamic>& H, Matrix<double, Dynamic, Dynamic>& g) {
    H.resize(2 * bezier_n_, 2 * bezier_n_);
    g.resize(2 * bezier_n_, 1);
    g.setZero();
}

bool OptBezierCurve::setBezierN(int n) {
    if(n > 2) {
        cout << "set bezier is " << n << endl;
        bezier_n_ = n;
        return true;
    }
    else {
        return false;
    }
}

int main(int argc, char** argv) {
    cout << "begin the program." << endl;
    ros::init(argc, argv, "opt_bezier_curve");
    OptBezierCurve test;
    return 0;
}