
#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#include "system_model.h"

using namespace Eigen;

class Magnetometer: public SystemModel{
    public:
        void InitMagnetometer(MatrixXd *R_bn, VectorXd *w_bn, double *dt);
        VectorXd RunModel(VectorXd x) override;
        // VectorXd RunModel(std::vector<State<VectorXd>> x) override;
    private:
        MatrixXd *_R_bn;
        VectorXd *_w_bn;
        double *_dt;
};

#endif