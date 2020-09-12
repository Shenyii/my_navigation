#include "opt_quintic_curve.h"

OptQuinticCurve::OptQuinticCurve() {
    cout << "init the opt quintic curve." << endl;
    tf_ = 2;
    min_v_ = 0.1;
    max_acc_ = 0.5;
    acc_constraint_num_ = 3;
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud>("/quintic_curve", 3);
}

OptQuinticCurve::~OptQuinticCurve() {}

void OptQuinticCurve::optSolveQuinticCurve(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta) {
    Matrix<double, 8, 8> M;
    Matrix<double, 4, 4> M1;
    M1.setZero();
    M1(0, 0) = 4. * tf_;          M1(0, 1) = 6. * pow(tf_, 2);  M1(0, 2) = 8. * pow(tf_, 3);    M1(0, 3) = 10. * pow(tf_, 4);
    M1(1, 0) = 6. * pow(tf_, 2);  M1(1, 1) = 12. * pow(tf_, 3); M1(1, 2) = 18. * pow(tf_, 4);   M1(1, 3) = 24. * pow(tf_, 5); 
    M1(2, 0) = 8. * pow(tf_, 3);  M1(2, 1) = 18. * pow(tf_, 4); M1(2, 2) = 28.8 * pow(tf_, 5);  M1(2, 3) = 40. * pow(tf_, 6); 
    M1(3, 0) = 10. * pow(tf_, 4); M1(3, 1) = 24. * pow(tf_, 5); M1(3, 2) = 40. * pow(tf_, 6);   M1(3, 3) = 400. / 7 * pow(tf_, 7); 
    M.setZero();
    M.block(0, 0, 4, 4) = M1;
    M.block(4, 4, 4, 4) = M1;
    //cout << M << endl;
    Matrix<double, 6, 6> M_qc;
    M_qc(0, 0) = 1; M_qc(0, 1) = 0;   M_qc(0, 2) = 0;           M_qc(0, 3) = 0;               M_qc(0, 4) = 0;                M_qc(0, 5) = 0;
    M_qc(1, 0) = 1; M_qc(1, 1) = tf_; M_qc(1, 2) = pow(tf_, 2); M_qc(1, 3) = pow(tf_, 3);     M_qc(1, 4) = pow(tf_, 4);      M_qc(1, 5) = pow(tf_, 5);
    M_qc(2, 0) = 0; M_qc(2, 1) = 1;   M_qc(2, 2) = 0;           M_qc(2, 3) = 0;               M_qc(2, 4) = 0;                M_qc(2, 5) = 0;
    M_qc(3, 0) = 0; M_qc(3, 1) = 1;   M_qc(3, 2) = 2 * tf_;     M_qc(3, 3) = 3 * pow(tf_, 2); M_qc(3, 4) = 4 * pow(tf_, 3);  M_qc(3, 5) = 5 * pow(tf_, 4);
    M_qc(4, 0) = 0; M_qc(4, 1) = 0;   M_qc(4, 2) = 2;           M_qc(4, 3) = 0;               M_qc(4, 4) = 0;                M_qc(4, 5) = 0;
    M_qc(5, 0) = 0; M_qc(5, 1) = 0;   M_qc(5, 2) = 2;           M_qc(5, 3) = 6 * tf_;         M_qc(5, 4) = 12 * pow(tf_, 2); M_qc(5, 5) = 20 * pow(tf_, 3);
    Matrix<double, 6, 6> in_M_qc = M_qc.inverse();
    Matrix<double, 8, 6> A;
    Matrix<double, 8, 1> B;
    A.setZero();
    A.block(0, 0, 4, 4) = in_M_qc.block(2, 2, 4, 4);
    // if(start_theta > 0) {
    //     A.block(4, 0, 4, 1) = fabs(tan(start_theta)) * in_M_qc.block(2, 2, 4, 1);
    // }
    // else {
    //     A.block(4, 0, 4, 1) = -fabs(tan(start_theta)) * in_M_qc.block(2, 2, 4, 1);
    // }
    // if(start_theta > 0) {
    //     A.block(4, 1, 4, 1) = fabs(tan(goal_theta)) * in_M_qc.block(2, 3, 4, 1);
    // }
    // else {
    //     A.block(4, 1, 4, 1) = -fabs(tan(goal_theta)) * in_M_qc.block(2, 3, 4, 1);
    // }
    A.block(4, 0, 4, 1) = tan(start_theta) * in_M_qc.block(2, 2, 4, 1);
    A.block(4, 1, 4, 1) = tan(goal_theta) * in_M_qc.block(2, 3, 4, 1);
    
    A.block(4, 4, 4, 2) = in_M_qc.block(2, 4, 4, 2);
    //cout << A << endl;
    Matrix<double, 8, 4> M3;
    M3.setZero();
    M3.block(0, 0, 4, 2) = in_M_qc.block(2, 0, 4, 2);
    M3.block(4, 2, 4, 2) = in_M_qc.block(2, 0, 4, 2);
    Matrix<double, 4, 1> M4;
    M4(0, 0) = start_x;
    M4(1, 0) = goal_x;
    M4(2, 0) = start_y;
    M4(3, 0) = goal_y;
    B = M3 * M4;

    Matrix<double, 6, 6> H;
    Matrix<double, 6, 1> g;
    H = A.transpose() * M * A;
    g = B.transpose() * M * A;
    Matrix<double, 2, 6> AA;
    AA.setZero();
    AA.block(0, 0, 2, 2) = MatrixXd::Identity(2, 2);
    Matrix<double, 2, 1> lb;
    Matrix<double, 2, 1> ub;
    if(fabs(start_theta) > PI / 2) {
        lb(0, 0) = -10000;
        ub(0, 0) = min_v_ * cos(start_theta);
    }
    else {
        lb(0, 0) = min_v_ * cos(start_theta);
        ub(0, 0) = 10000;
    }
    if(fabs(goal_theta) > PI / 2) {
        lb(1, 0) = -10000;
        ub(1, 0) = min_v_ * cos(goal_theta);
    }
    else {
        lb(1, 0) = min_v_ * cos(goal_theta);
        ub(1, 0) = 10000;
    }
    //Matrix<double, 6, 1> M5 = -H.inverse() * g;
    Matrix<double, 6, 1> M5 = solveProblem(H, g, AA, lb, ub);

    Matrix<double, 6, 2> M_print;
    M_print.block(0, 0, 6, 1) = M5;
    M_print.block(0, 1, 6, 1) = -H.inverse() * g;
    cout << M_print << endl;

    Matrix<double, 6, 1> X;
    X(0, 0) = start_x;
    X(1, 0) = goal_x;
    X.block(2, 0, 4, 1) = M5.block(0, 0, 4, 1);
    Matrix<double, 6, 1> Y;
    Y(0, 0) = start_y;
    Y(1, 0) = goal_y;
    Y(2, 0) = tan(start_theta) * M5(0, 0);
    Y(3, 0) = tan(goal_theta) * M5(1, 0);
    Y.block(4, 0, 2, 1) = M5.block(4, 0, 2, 1);
    //cout << X.transpose() << endl << Y.transpose() << endl;

    Matrix<double, 6, 1> x_param = in_M_qc * X;
    Matrix<double, 6, 1> y_param = in_M_qc * Y;

    points_.header.frame_id = "map";
    points_.points.clear();
    for(double t = 0; t < tf_; t += 0.1) {
        geometry_msgs::Point32 point;
        point.x = x_param(0, 0) + x_param(1, 0) * t + x_param(2, 0) * pow(t, 2) + x_param(3, 0) * pow(t, 3) + x_param(4, 0) * pow(t, 4) + x_param(5, 0) * pow(t, 5);
        point.y = y_param(0, 0) + y_param(1, 0) * t + y_param(2, 0) * pow(t, 2) + y_param(3, 0) * pow(t, 3) + y_param(4, 0) * pow(t, 4) + y_param(5, 0) * pow(t, 5);
        point.z = 0;
        points_.points.push_back(point);
    }
    pub_points_.publish(points_);
}

void OptQuinticCurve::optSolveQuinticCurve2(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta) {
    Matrix<double, 12, 12> H;
    H.setZero();
    Matrix<double, 6, 6> M1;
    Matrix<double, 6, 1> M2;
    M2(0, 0) = 0;
    M2(1, 0) = 0;
    M2(2, 0) = 2;
    M2(3, 0) = 6 * tf_;
    M2(4, 0) = 12 * pow(tf_, 2);
    M2(5, 0) = 20 * pow(tf_, 3);
    M1 = M2 * M2.transpose();
    H.block(0, 0, 6, 6) = M1;
    H.block(6, 6, 6, 6) = M1;
    //cout << H << endl;
    Matrix<double, 12, 1> g;
    g.setZero();
    
    //cout << M << endl;
    Matrix<double, 6, 6> M_qc;
    M_qc(0, 0) = 1; M_qc(0, 1) = 0;   M_qc(0, 2) = 0;           M_qc(0, 3) = 0;               M_qc(0, 4) = 0;                M_qc(0, 5) = 0;
    M_qc(1, 0) = 1; M_qc(1, 1) = tf_; M_qc(1, 2) = pow(tf_, 2); M_qc(1, 3) = pow(tf_, 3);     M_qc(1, 4) = pow(tf_, 4);      M_qc(1, 5) = pow(tf_, 5);
    M_qc(2, 0) = 0; M_qc(2, 1) = 1;   M_qc(2, 2) = 0;           M_qc(2, 3) = 0;               M_qc(2, 4) = 0;                M_qc(2, 5) = 0;
    M_qc(3, 0) = 0; M_qc(3, 1) = 1;   M_qc(3, 2) = 2 * tf_;     M_qc(3, 3) = 3 * pow(tf_, 2); M_qc(3, 4) = 4 * pow(tf_, 3);  M_qc(3, 5) = 5 * pow(tf_, 4);
    M_qc(4, 0) = 0; M_qc(4, 1) = 0;   M_qc(4, 2) = 2;           M_qc(4, 3) = 0;               M_qc(4, 4) = 0;                M_qc(4, 5) = 0;
    M_qc(5, 0) = 0; M_qc(5, 1) = 0;   M_qc(5, 2) = 2;           M_qc(5, 3) = 6 * tf_;         M_qc(5, 4) = 12 * pow(tf_, 2); M_qc(5, 5) = 20 * pow(tf_, 3);
    Matrix<double, Dynamic, Dynamic> A;
    A.resize(8 + 2 * acc_constraint_num_, 12);
    A.setZero();
    A.block(0, 0, 2, 6) = M_qc.block(0, 0, 2, 6);
    A.block(2, 6, 2, 6) = M_qc.block(0, 0, 2, 6);
    A.block(4, 0, 1, 6) = M_qc.block(2, 0, 1, 6) * sin(start_theta);
    A.block(4, 6, 1, 6) = M_qc.block(2, 0, 1, 6) * (-cos(start_theta));
    A.block(5, 0, 1, 6) = M_qc.block(3, 0, 1, 6) * sin(goal_theta);
    A.block(5, 6, 1, 6) = M_qc.block(3, 0, 1, 6) * (-cos(goal_theta));
    A.block(6, 0, 1, 6) = M_qc.block(2, 0,1, 6);
    A.block(7, 0, 1, 6) = M_qc.block(3, 0,1, 6);
    for(int i = 0; i < acc_constraint_num_; i++) {
        double t = i * tf_ / (acc_constraint_num_ - 1);
        A.block(8 + 2 * i, 0, 1, 6) << 0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);
        A.block(9 + 2 * i, 6, 1, 6) << 0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);
    }
    cout << A << endl;
    Matrix<double, Dynamic, Dynamic> lb;
    Matrix<double, Dynamic, Dynamic> ub;
    lb.resize(8 + 2 * acc_constraint_num_, 1);
    ub.resize(8 + 2 * acc_constraint_num_, 1);
    lb(0 ,0) = start_x - 0.0001; ub(0, 0) = start_x + 0.0001;
    lb(1 ,0) = goal_x - 0.0001;  ub(1, 0) = goal_x + 0.0001;
    lb(2 ,0) = start_y - 0.0001; ub(2, 0) = start_y + 0.0001;
    lb(3 ,0) = goal_y - 0.0001;  ub(3, 0) = goal_y + 0.0001;
    lb(4 ,0) = -0.001;    ub(4, 0) = 0.001;
    lb(5 ,0) = -0.001;    ub(5, 0) = 0.001;
    if(fabs(start_theta) > PI / 2) {
        lb(6, 0) = -100; ub(6, 0) = min_v_ * cos(start_theta);
    }
    else {
        lb(6 ,0) = min_v_ * cos(start_theta);    ub(6, 0) = 100;
    }
    if(fabs(goal_theta) > PI / 2) {
        lb(7, 0) = -100; ub(7, 0) = min_v_ * cos(goal_theta);
    }
    else {
        lb(7 ,0) = min_v_ * cos(goal_theta);    ub(7, 0) = 100;
    }
    for(int i = 0; i < acc_constraint_num_; i++) {
        lb(8 + 2 * i, 0) = -max_acc_; ub(8 + 2 * i, 0) = max_acc_;
        lb(9 + 2 * i, 0) = -max_acc_; ub(9 + 2 * i, 0) = max_acc_;
    }

    Matrix<double, Dynamic, 1> curve_param = solveProblem(H, g, A, lb, ub);
    
    Matrix<double, 6, 1> x_param;
    Matrix<double, 6, 1> y_param;
    x_param.block(0, 0, 6, 1) = curve_param.block(0, 0, 6, 1);
    y_param.block(0, 0, 6, 1) = curve_param.block(6, 0, 6 ,1);

    points_.header.frame_id = "map";
    points_.points.clear();
    for(double t = 0; t < tf_; t += 0.1) {
        geometry_msgs::Point32 point;
        point.x = x_param(0, 0) + x_param(1, 0) * t + x_param(2, 0) * pow(t, 2) + x_param(3, 0) * pow(t, 3) + x_param(4, 0) * pow(t, 4) + x_param(5, 0) * pow(t, 5);
        point.y = y_param(0, 0) + y_param(1, 0) * t + y_param(2, 0) * pow(t, 2) + y_param(3, 0) * pow(t, 3) + y_param(4, 0) * pow(t, 4) + y_param(5, 0) * pow(t, 5);
        point.z = 0;
        points_.points.push_back(point);
    }
    pub_points_.publish(points_);
}

Matrix<double, Dynamic, 1> OptQuinticCurve::solveProblem(Matrix<double, Dynamic, Dynamic> H, Matrix<double, Dynamic, Dynamic> g,
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

void OptQuinticCurve::matrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) {
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

int OptQuinticCurve::upperMatrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) {
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