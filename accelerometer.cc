#include "accelerometer.h"

using namespace Eigen;

void Accelerometer::InitAccelerometer(MatrixXd *R_bn, VectorXd *w_bn, VectorXd *b_a, double *dt) {
    _R_bn = R_bn;
    _w_bn = w_bn;
    _b_a = b_a;
    _dt = dt;
    // _g_n << 0, 0, 9.81;
}

VectorXd Accelerometer::RunModel(VectorXd x) {
    VectorXd y(3);
    VectorXd g_n(3);
    // VectorXd b_a = GetSubspace(x, 0, 2);
    // VectorXd b_a = VectorXd::Zero(3);
    g_n << 0, 0, 9.81;
    y = RotateVector(x.head(3))*(*_R_bn)*(-1*g_n) + (*_b_a);
    // y = RotateVector(x.head(3))*(*_R_bn)*RotateVector((*_dt)*(*_w_bn-x.tail(3)))*(-1*g_n);
    return y;
}

// VectorXd Accelerometer::RunModel(std::vector<State<VectorXd>> x) {
//     VectorXd b_a = *_b_a;
//     VectorXd eta = VectorXd::Zero(3);
    
//     for (int i = 0; i < x.size(); i++) {
//         if (x[i].GetName() == "bias") {
//             // b_a = x[i].GetValue();
//         } 
//         else if (x[i].GetName() == "eta") {
//             // eta = x[i].GetValue();
//         }
//     }
//     VectorXd y = RotateVector(eta)*(*_R_bn)*(-1*_g_n) + b_a;
//     return y;
// }