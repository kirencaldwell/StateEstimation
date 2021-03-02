
#include "magnetometer.h"

using namespace Eigen;

void Magnetometer::InitMagnetometer(MatrixXd *R_bn, VectorXd *w_bn, double *dt) {
    _R_bn = R_bn;
    _w_bn = w_bn;
    _dt = dt;
}

VectorXd Magnetometer::RunModel(VectorXd x) {
    // std::cout << "R_m = " << (*_R_bn) << "\n\n";
    VectorXd y(3);
    VectorXd m_n(3);
    m_n << 1, 0, 0;
    y = RotateVector(x.head(3))*(*_R_bn)*(m_n);
    // y = RotateVector(x.head(3))*(*_R_bn)*RotateVector((*_dt)*(*_w_bn-x.tail(3)))*(m_n);
    return y;
}