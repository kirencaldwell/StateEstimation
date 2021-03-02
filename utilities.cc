#include "utilities.h"

MatrixXd skew(VectorXd t) {
    MatrixXd t_hat(3,3);
    t_hat << 0, -t(2), t(1),
    t(2), 0, -t(0),
    -t(1), t(0), 0;
    return t_hat;
}

VectorXd vee(MatrixXd a) {
    VectorXd x(3);
    x << -a(1,2), a(0,2), -a(0,1);
    return x;
}

MatrixXd RotateVector(VectorXd x) {
    MatrixXd R;
    MatrixXd a(3, 3);
    a = skew(x);
    R = a.exp();
    return R;
}

VectorXd RotationToVector(MatrixXd R) {
    MatrixXd a(3, 3);
    a = R.log();
    return vee(a);
}

MatrixXd AssignSubspace(MatrixXd A, int row, int col, MatrixXd a) {
    for (int i = 0; i < a.cols(); i++) {
        for (int j = 0; j < a.rows(); j++) {
            A(row+j, col+i) = a(j,i);
        }
    }
    return A;
}

MatrixXd GetSubspace(MatrixXd A, int rowi, int rowf, int coli, int colf) {
    MatrixXd a = MatrixXd(rowf-rowi+1, colf-coli+1);
    for (int i = 0; i < a.rows(); i++) {
        for (int j = 0; j < a.cols(); j++) {
            a(i,j) = A(i+rowi, j+coli);
        }
    }
    return a;
}
VectorXd GetSubspace(VectorXd a, int s, int f) {
    VectorXd b = VectorXd(f-s+1);
    for (int i = 0; i < b.size(); i++) {
        b(i) = a(i+s);
    }
    return b;
}

std::string CreateLoggingString(double x) {
    std::string out;
    std::string xx;
    xx = std::to_string(x) + ", ";
    out.append(xx);
    return out;
}
std::string CreateLoggingString(VectorXd x) {
    std::string out;
    std::string xx;
    for (int i = 0; i < x.size(); i++) {
        xx = std::to_string(x(i)) + ", ";
        out.append(xx);
    }
    return out;
}
std::string CreateLoggingString(MatrixXd x) {
    std::string out;
    std::string xx;
    for (int i = 0; i < x.cols(); i++) {
        for (int j = 0; j < x.rows(); j++) {
            xx = std::to_string(x(j,i)) + ", ";
            out.append(xx);
        }
    }
    return out;
}

void PrintTimeUpdate(double t, MatrixXd R, int K) {
    static int k(0);
    if (k % K == 0) {
        std::cout << "t = " << t << ", R = \n" << R << "\n\n";
    }
    k++;
}