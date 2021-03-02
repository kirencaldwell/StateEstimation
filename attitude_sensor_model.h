

#ifndef ATTITUDE_SENSOR_MODEL_H_
#define ATTITUDE_SENSOR_MODEL_H_

#include "system_model.h"
#include "utilities.h"
#include "unsupported/Eigen/MatrixFunctions"

using namespace Eigen;

class AttitudeSensorModel: public SystemModel{
    public:
        void InitAttitudeSensorModel(MatrixXd *R_bn, VectorXd *w_bn, double *dt);
        VectorXd GetNoisyOutput(VectorXd x) override; 
        VectorXd RunModel(VectorXd x) override;
        void ApplyAttitudeCorrection(VectorXd x);
    private:
        MatrixXd *_R_bn;
        VectorXd *_w_bn;
        double *_dt;
};

#endif