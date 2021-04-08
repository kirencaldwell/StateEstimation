
#include "uncalibrated_accelerometer.h"

using namespace Eigen;

void UncalibratedAccelerometer::InitUncalibratedAccelerometer(MatrixXd *R_bn, VectorXd *a_nn, VectorXd *b_a, MatrixXd *C_a) {
    _R_bn = R_bn;
    _a_nn = a_nn;
    _b_a = b_a;
    _C_a = C_a;
}

VectorXd UncalibratedAccelerometer::RunModel(VectorXd x) {
    VectorXd y(3);
    VectorXd g_n(3);
    g_n << 0, 0, 9.81;
    y = (*_R_bn)*((*_a_nn)-1*g_n) + x.tail(3);
    return y;
}
