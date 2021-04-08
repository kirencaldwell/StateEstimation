#ifndef TRANSLATIONAL_MODEL_H
#define TRANSLATIONAL_MODEL_H

#include "system_model.h"

using namespace Eigen;

class TranslationalModel: public SystemModel{
    public:
        void InitTranslationalModel(VectorXd *y_a, MatrixXd *R_bn, double *dt);
        VectorXd RunModel(VectorXd x) override;
    private:
        VectorXd *_y_a;
        MatrixXd *_R_bn;
        double *_dt;
};


#endif