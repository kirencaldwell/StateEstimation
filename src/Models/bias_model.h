
#ifndef BIAS_MODEL_H_
#define BIAS_MODEL_H_

#include "system_model.h"

using namespace Eigen;

class BiasModel: public SystemModel{
    public:
        void InitBiasModel();
        VectorXd RunModel(VectorXd x) override;
    private:
};

#endif