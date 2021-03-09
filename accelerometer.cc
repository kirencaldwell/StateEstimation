#include "accelerometer.h"

using namespace Eigen;

void Accelerometer::InitAccelerometer(MatrixXd *R_bn, VectorXd *a_nn, VectorXd *w_bn, VectorXd *b_a, double *dt) {
    _R_bn = R_bn;
    _a_nn = a_nn;
    _w_bn = w_bn;
    _b_a = b_a;
    _dt = dt;
}

VectorXd Accelerometer::RunModel(VectorXd x) {
    VectorXd y(3);
    VectorXd g_n(3);
    g_n << 0, 0, 9.81;
    y = RotateVector(x.head(3))*(*_R_bn)*((*_a_nn)-1*g_n) + (*_b_a);
    return y;
}
