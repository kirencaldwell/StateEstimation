
#ifndef UNCALIBRATED_ACCELEROMETER_H_
#define UNCALIBRATED_ACCELEROMETER_H_

#include "system_model.h"
#include "utilities.h"
#include "unsupported/Eigen/MatrixFunctions"
#include "state.h"
#include <vector>

using namespace Eigen;

class UncalibratedAccelerometer: public SystemModel{
    public:
        void InitUncalibratedAccelerometer(MatrixXd *R_bn, VectorXd *a_nn, VectorXd *b_a, MatrixXd *C_a);
        VectorXd RunModel(VectorXd x) override;
    private:
        MatrixXd *_R_bn;
        VectorXd *_a_nn;
        VectorXd *_b_a;
        MatrixXd *_C_a;
};

#endif