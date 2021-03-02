
#ifndef ATTITUDE_DEVIATION_MODEL_H_
#define ATTITUDE_DEVIATION_MODEL_H_

#include "system_model.h"
#include "utilities.h"
#include "unsupported/Eigen/MatrixFunctions"

using namespace Eigen;

class AttitudeDeviationModel: public SystemModel{
    public:
        void InitAttitudeDeviationModel(MatrixXd *R_bn, VectorXd *w_bn, VectorXd *b_g, double *dt);
        VectorXd GetNoisyOutput(VectorXd x) override; 
        VectorXd RunModel(VectorXd x) override;
        MatrixXd GetJacobian(VectorXd x, double h) override;
        MatrixXd GetVariance() override;
    private:
        MatrixXd *_R_bn;
        VectorXd *_w_bn;
        VectorXd *_b_g;
        double *_dt;
        VectorXd DeviationUpdate(VectorXd x);
        VectorXd DeviationNoise(VectorXd x);
        MatrixXd GetNoiseJacobian(VectorXd x, double h);
};

#endif