
#ifndef BIAS_MODEL_H_
#define BIAS_MODEL_H_

#include "system_model.h"
#include "utilities.h"
#include "unsupported/Eigen/MatrixFunctions"

using namespace Eigen;

class BiasModel: public SystemModel{
    public:
        void InitBiasModel();
        VectorXd RunModel(VectorXd x) override;
    private:
};

#endif