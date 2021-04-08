#include "translational_model.h"


void TranslationalModel::InitTranslationalModel(VectorXd *y_a, MatrixXd *R_bn, double *dt) {
    _y_a = y_a;
    _R_bn = R_bn;
    _dt = dt;
}

VectorXd TranslationalModel::RunModel(VectorXd x) {
    VectorXd x_dot(6);
    VectorXd g_n(3);
    g_n << 0, 0, 9.81;
    VectorXd a_nn = (*_R_bn).transpose()*((*_y_a) + x.tail(3)) + g_n;
    x_dot << GetSubspace(x, 3, 5), a_nn+x.tail(3);

    return x.head(6) + (*_dt)*x_dot;
}