#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#include "system_model.h"
#include "utilities.h"
#include "unsupported/Eigen/MatrixFunctions"

using namespace Eigen;

class Gyroscope: public SystemModel{
    public:
        void InitGyroscope(VectorXd *w_bn, VectorXd *b_g);
        VectorXd RunModel(VectorXd x) override;
    private:
        VectorXd *_w_bn;
        VectorXd *_b_g;
};
#endif