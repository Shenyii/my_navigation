#include "opt_bezier_curve.h"

OptBezierCurve::OptBezierCurve() {
    bezier_n_ = 5;
    min_v_ = 0.1;
    max_v_ = 100;
    min_acc_ = -0.5;
    max_acc_ = 1;
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud>("/bezier_curve", 3);
}

OptBezierCurve::~OptBezierCurve() {}

bool OptBezierCurve::getOptBezierCurve(double start_x, double start_y, double start_theta, double end_x, double end_y, double end_theta) {
    Matrix<double, Dynamic, Dynamic> H, g, A, lb, ub;
    setMatrixHg(H, g);
    setContraint(start_x, start_y, start_theta, end_x, end_y, end_theta, A, lb, ub);
    //cout << A << endl << lb << endl << ub << endl;

    //H.setZero();

    Matrix<double, Dynamic, Dynamic> P = solveProblem(H, g, A, lb, ub);
    if(P.rows() == 0) {
        return false;
    }
    //cout << P.transpose() << endl;
    //cout << A << endl;
    for(int i = 0; i < bezier_n_ + 1; i++) {
        cout << P(2 * i, 0) << ", " << P(2 * i + 1, 0) << endl;
    }
    
    sensor_msgs::PointCloud points;
    points.header.frame_id = "map";
    points.points.clear();
    for(double t = 0; t < 1; t += 0.02) {
        geometry_msgs::Point32 point;
        double x = 0;
        double y = 0;
        for(int i = 0; i < P.size() / 2; i++) {
            x += P(2 * i + 0) * C_n_m(bezier_n_, i) * pow(1 - t, bezier_n_ - i) * pow(t, i);
            y += P(2 * i + 1) * C_n_m(bezier_n_, i) * pow(1 - t, bezier_n_ - i) * pow(t, i);
        }
        point.x = x;
        point.y = y;
        point.z = 0;
        //cout << x << ", " << y << endl;
        points.points.push_back(point);
    }
    for(int i = 0; i < bezier_n_ + 1; i++) {
        geometry_msgs::Point32 point;
        point.x = P(2 * i, 0);
        point.y = P(2 * i + 1, 0);
        point.z = 0;
        points.points.push_back(point);
    }
    pub_points_.publish(points);

    for(double t = 0; t < 1; t += 0.02) {
        double ax = 0;
        double ay = 0;
        for(int i = 0; i < P.size() / 2 - 2; i++) {
            ax += (P(2*i+4) - 2 * P(2*i+2) + P(2*i+0)) * bezier_n_ * (bezier_n_ - 1) * C_n_m(bezier_n_ - 2, i) * pow(1 - t, bezier_n_ - 2 - i) * pow(t, i);
            ay += (P(2*i+5) - 2 * P(2*i+3) + P(2*i+1)) * bezier_n_ * (bezier_n_ - 1) * C_n_m(bezier_n_ - 2, i) * pow(1 - t, bezier_n_ - 2 - i) * pow(t, i);
        }
        double ax0 = 0;
        double ay0 = 1;
        int i = t / 0.02;
        double vx0 = (points.points[i + 1].x - points.points[i].x) / 0.02;
        double vy0 = (points.points[i + 1].y - points.points[i].y) / 0.02;
        ax0 = (points.points[i + 2].x - points.points[i + 1].x * 2 + points.points[i].x) / 0.0004;
        ay0 = (points.points[i + 2].y - points.points[i + 1].y * 2 + points.points[i].y) / 0.0004;
        cout << "vel: " << vx0 << ", " << vy0 << ", acc: " << ax << ", " << ay << ", " << ax0 << ", " << ay0 << endl;
    }

    return true;
}

void OptBezierCurve::setMatrixHg(Matrix<double, Dynamic, Dynamic>& H, Matrix<double, Dynamic, Dynamic>& g) {
    H.resize(2 * bezier_n_ + 2, 2 * bezier_n_ + 2);
    H.setZero();
    Matrix<double, Dynamic, Dynamic> M1, M2;
    M1.resize(2 * bezier_n_ - 2, 2 * bezier_n_ - 2);
    M2.resize(2 * bezier_n_ - 2, 2 * bezier_n_ + 2);
    M2.setZero();
    for(int i = 0; i < M2.rows(); i++) {
        for(int j = 0; j < M2.cols(); j++) {
            if(i == j) M2(i, j) = 1;
            if(i + 2 == j) M2(i, j) = -2;
            if(i + 4 == j) M2(i, j) = 1;
        }
    }
    M1.setZero();
    for(int i = 0; i < M1.rows(); i++) {
        for(int j = 0; j < M1.cols(); j++) {
            if((i + j) % 2 == 0) {
                M1(i, j) = C_n_m(bezier_n_ - 2, i / 2) * C_n_m(bezier_n_ - 2, j / 2) * integralSpecialFun(2 * bezier_n_ - 4 - i / 2 - j / 2, i / 2 + j / 2);
            }
        }
    }
    //cout << M1 << endl;
    

    H = M2.transpose() * M1 * M2;

    g.resize(2 * bezier_n_ + 2, 1);
    g.setZero();
}

void OptBezierCurve::setContraint(double start_x, double start_y, double start_theta, double end_x, double end_y, double end_theta,
                                  Matrix<double, Dynamic, Dynamic>& A, Matrix<double, Dynamic, Dynamic>& lb, Matrix<double, Dynamic, Dynamic>& ub) {
    A.resize(2 * bezier_n_ + 6, 2 * bezier_n_ + 2);
    lb.resize(2 * bezier_n_ + 6, 1);
    ub.resize(2 * bezier_n_ + 6, 1);
    A.setZero();
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(2, 2 * bezier_n_) = 1;
    A(3, 2 * bezier_n_ + 1) = 1;
    A(4, 0) = sin(start_theta); A(4, 1) = -cos(start_theta); A(4, 2) = -sin(start_theta); A(4, 3) = cos(start_theta);
    A(5, 2 * bezier_n_ - 2) = sin(end_theta); A(5, 2 * bezier_n_ - 1) = -cos(end_theta); A(5, 2 * bezier_n_) = -sin(end_theta); A(5, 2 * bezier_n_ + 1) = cos(end_theta);
    A(6, 0) = -1.0 * bezier_n_; A(6, 2) = 1.0 * bezier_n_;
    A(7, 2 * bezier_n_ - 2) = -1.0 * bezier_n_; A(7, 2 * bezier_n_) = 1.0 * bezier_n_;
    Matrix<double, Dynamic, Dynamic> M1;
    M1.resize(2 * bezier_n_ - 2, 2 * bezier_n_ + 2);
    M1.setZero();
    for(int i = 0; i < M1.rows(); i++) {
        for(int j = 0; j < M1.cols(); j++) {
            if(i == j) M1(i, j) = 1.0 * bezier_n_ * (bezier_n_ - 1);
            if(i + 2 == j) M1(i, j) = -2.0 * bezier_n_ * (bezier_n_ - 1);
            if(i + 4 == j) M1(i, j) = 1.0 * bezier_n_ * (bezier_n_ - 1);
        }
    }
    A.block(8, 0, 2 * bezier_n_ - 2, 2 * bezier_n_ + 2) = M1;

    lb(0, 0) = start_x - 0.0001; ub(0, 0) = start_x + 0.0001;
    lb(1, 0) = start_y - 0.0001; ub(1, 0) = start_y + 0.0001;
    lb(2, 0) = end_x - 0.0001;   ub(2, 0) = end_x + 0.0001;
    lb(3, 0) = end_y - 0.0001;   ub(3, 0) = end_y + 0.0001;
    lb(4, 0) = -0.0001;          ub(4, 0) = 0.0001;
    lb(5, 0) = 0.0001;           ub(5, 0) = 0.0001;
    if(fabs(start_theta) > PI / 2) {
        lb(6, 0) = -max_v_; ub(6, 0) = min_v_ * cos(start_theta);
    }
    else {
        lb(6, 0) = min_v_ * cos(start_theta); ub(6, 0) = max_v_;
    }
    if(fabs(end_theta) > PI / 2) {
        lb(7, 0) = -max_v_; ub(7, 0) = min_v_ * cos(end_theta);
    }
    else {
        lb(7, 0) = min_v_ * cos(end_theta); ub(7, 0) = max_v_;
    }
    for(int i = 8; i < 2 * bezier_n_ + 6; i++) {
        lb(i, 0) = -max_acc_; ub(i, 0) = max_acc_;
    }
}

double OptBezierCurve::integralSpecialFun(int a, int b) {
    double ans = 0;
    double symbol_flag = -1;
    if(a == 0) {
        return 1.0 / (b + 1.0);
    }
    for(int i = 0; i < a + 1; i++) {
        symbol_flag *= -1;
        ans += symbol_flag * C_n_m(a, i) / (b + i + 1.0);
    }

    return ans;
}

double OptBezierCurve::C_n_m(int n, int m) {
    double ans = 1;
    if(m > n) {
        cout << "C_n_m is error." << endl;
        return 1;
    }
    for(int i = 0; i < m; i++) {
        ans *= (n - i) * 1.0 / (i + 1.0);
    }

    return ans;
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

Matrix<double, Dynamic, 1> OptBezierCurve::solveProblem(Matrix<double, Dynamic, Dynamic> H, Matrix<double, Dynamic, Dynamic> g,
                                                         Matrix<double, Dynamic, Dynamic> A,Matrix<double, Dynamic, Dynamic> lb, Matrix<double, Dynamic, Dynamic> ub) {
    Matrix<double, Dynamic, 1> ans;
    ans.resize(H.rows(), 1);

    c_float* P_x;
    c_int   P_nnz;
    c_int*   P_i;
    c_int*   P_p;
    c_float* q;
    c_float* A_x;
    c_int   A_nnz;
    c_int*   A_i;
    c_int*   A_p;
    c_float* l;
    c_float* u;
    c_int n = H.rows();
    c_int m = A.rows();
    upperMatrixToCsc(&P_x, P_nnz, &P_i, &P_p, H);
    matrixToCsc(&A_x, A_nnz, &A_i, &A_p, A);
    q = new c_float[n];
    for(int i = 0; i < n; i++) q[i] = g(i, 0);
    l = new c_float[m];
    u = new c_float[m];
    for(int i = 0; i < m; i++) {
        l[i] = lb(i, 0);
        u[i] = ub(i, 0);
    }

    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));
    OSQPSolution test1;

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

    // Define solver settings as default
    if (settings) osqp_set_default_settings(settings);

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    if(work->info->status_val != 1) {
        cout << "Can't solve the problam." << endl;
        ans.resize(0, 1);
        return ans;
    }

    for(int i = 0; i < ans.rows(); i++) {
        ans(i, 0) = work->solution->x[i];
    }

    // Clean workspace
    osqp_cleanup(work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings)  c_free(settings);

    delete [] P_x;
    delete [] P_i;
    delete [] P_p;
    delete [] q;
    delete [] A_x;
    delete [] A_i;
    delete [] A_p;
    delete [] l;
    delete [] u;

    return ans;
}

void OptBezierCurve::matrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) {
    M_nnz = 0;
    for(int j = 0; j < M.cols(); j++) {
        for(int i = 0; i < M.rows(); i++) {
            if(M(i, j) != 0) {
                M_nnz++;
            }
        }
    }
    *M_x = new c_float[M_nnz];
    int ptr1 = 0;
    for(int j = 0; j < M.cols(); j++) {
        for(int i = 0; i < M.rows(); i++) {
            if(M(i, j) != 0) {
                (*M_x)[ptr1] = M(i, j);
                ptr1++;
            }
        }
    }
    int ptr2 = 0;
    *M_i = new c_int[M_nnz];
    *M_p = new c_int[M.cols() + 1];
    (*M_p)[0] = 0;
    for(int j = 0; j < M.cols(); j++) {
        int ptr3 = 0;
        for(int i = 0; i < M.rows(); i++) {
            if(M(i, j) != 0) {
                (*M_i)[ptr2] = i;
                ptr2++;
                ptr3++;
            }
        }
        (*M_p)[j + 1] = (*M_p)[j] + ptr3;
    }
}

int OptBezierCurve::upperMatrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) {
    if(M.cols() != M.rows()) {
        cout << "The matrix isn't symmetry." << endl << "The program will error!" << endl;
        return 0;
    }
    for(int j = 0; j < M.cols(); j++) {
        for(int i = 0; i < M.rows(); i++) {
            if(i > j) {
                M(i, j) = 0;
            }
        }
    }
    matrixToCsc(M_x, M_nnz, M_i, M_p, M);
    return 1;
}

// int main(int argc, char** argv) {
//     cout << "begin the program." << endl;
//     ros::init(argc, argv, "opt_bezier_curve");
//     OptBezierCurve test;
//     test.getOptBezierCurve(0, 0, -1.4, 1, 1, 1);
//     ros::spin();
//     return 0;
// }