#include "gyroscope.h"

using namespace Eigen;

void Gyroscope::InitGyroscope(VectorXd *w_bn, VectorXd *b_g) {
    _w_bn = w_bn;
    _b_g = b_g;
}

VectorXd Gyroscope::RunModel(VectorXd x) {
    VectorXd y;
    VectorXd b_g = GetSubspace(x, 3, 5);
    return (*_w_bn) - b_g;
}