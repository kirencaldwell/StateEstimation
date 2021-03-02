
#include "bias_model.h"

using namespace Eigen;

void BiasModel::InitBiasModel() {
}

VectorXd BiasModel::RunModel(VectorXd x) {
    return x.tail(3);
}
