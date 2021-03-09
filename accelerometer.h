#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "system_model.h"
#include "utilities.h"
#include "unsupported/Eigen/MatrixFunctions"
#include "state.h"
#include <vector>

using namespace Eigen;

class Accelerometer: public SystemModel{
    public:
        void InitAccelerometer(MatrixXd *R_bn, VectorXd *a_nn, VectorXd *w_bn, VectorXd *b_a, double *dt);
        VectorXd RunModel(VectorXd x) override;
        // VectorXd RunModel(std::vector<State<VectorXd>> x) override;
    private:
        MatrixXd *_R_bn;
        VectorXd *_a_nn;
        VectorXd *_w_bn;
        VectorXd *_b_a;
        double *_dt;
        // VectorXd _g_n;
};

#endif