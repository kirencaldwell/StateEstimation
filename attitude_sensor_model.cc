
#include "attitude_sensor_model.h"

using namespace Eigen;

void AttitudeSensorModel::InitAttitudeSensorModel(MatrixXd *R_bn, VectorXd *w_bn, double *dt) {
    _R_bn = R_bn;
    _w_bn = w_bn;
    _dt = dt;
}

VectorXd AttitudeSensorModel::GetNoisyOutput(VectorXd x) {
    return RunModel(x);
}

VectorXd AttitudeSensorModel::RunModel(VectorXd x) {
    VectorXd y(3);
    // std::cout << "Rr = " << (*_R_bn) << "\n\n";
    y << 0, 0, 0; 
    *_R_bn = (*_R_bn) * RotateVector((*_dt)*((*_w_bn)-x.tail(3)));
    return y;
}

void AttitudeSensorModel::ApplyAttitudeCorrection(VectorXd x) {
    *_R_bn = RotateVector(x.head(3)) * (*_R_bn);
}