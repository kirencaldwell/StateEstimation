
#include "attitude_deviation_model.h"

using namespace Eigen;

void AttitudeDeviationModel::InitAttitudeDeviationModel(MatrixXd *R_bn, VectorXd *w_bn, VectorXd *b_g, double *dt) {
    _R_bn = R_bn;
    _w_bn = w_bn;
    _dt = dt;
    _b_g = b_g;
}

VectorXd AttitudeDeviationModel::GetNoisyOutput(VectorXd x) {
    return RunModel(x);
}

MatrixXd AttitudeDeviationModel::GetVariance() {
    MatrixXd F;
    VectorXd e(3);
    e << 0, 0, 0;
    F = GetNoiseJacobian(e, 0.001);
    MatrixXd Q;
    Q = F*_R*F.transpose();
    return Q; 
}

VectorXd AttitudeDeviationModel::RunModel(VectorXd x) {
    VectorXd y(3);
    // y = DeviationUpdate(x);
    y << 0, 0, 0;
    return y;
}

VectorXd AttitudeDeviationModel::DeviationNoise(VectorXd x) {
    VectorXd y(3);
    VectorXd eta(3);
    eta << 0, 0, 0;
    MatrixXd Rp = (*_R_bn) * RotateVector((*_dt)*(*_w_bn));
    MatrixXd Rp_eta = RotateVector(eta) * (*_R_bn) * RotateVector((*_dt)*(*_w_bn-x)) * Rp.transpose();
    y = RotationToVector(Rp_eta);
    return y;
}

MatrixXd AttitudeDeviationModel::GetNoiseJacobian(VectorXd x, double h) {

    VectorXd y0 = DeviationNoise(x);
    int m = y0.size();
    int n = x.size();
    MatrixXd H(m, n);

    for (int i = 0; i < n; i++) {
        VectorXd x0 = x;
        x0(i) = x0(i) + h;
        H.col(i) = (DeviationNoise(x0) - y0) / h;
    } 
     
    return H;
}

MatrixXd AttitudeDeviationModel::GetJacobian(VectorXd x, double h) {

    VectorXd y0 = DeviationUpdate(x);
    int m = y0.size();
    int n = x.size();
    MatrixXd H(m, n);

    for (int i = 0; i < n; i++) {
        VectorXd x0 = x;
        x0(i) = x0(i) + h;
        H.col(i) = (DeviationUpdate(x0) - y0) / h;
    } 
     
    return H;
}

VectorXd AttitudeDeviationModel::DeviationUpdate(VectorXd x) {
    VectorXd y(3);
    VectorXd b_g = GetSubspace(x, 3, 5);
    MatrixXd Rp = (*_R_bn) * RotateVector((*_dt)*(*_w_bn));
    MatrixXd Rp_eta = RotateVector(x.head(3)) * (*_R_bn) * RotateVector((*_dt)*((*_w_bn) - b_g)) * Rp.transpose();
    y = RotationToVector(Rp_eta);
    
    return y;
}