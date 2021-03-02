#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "../Models/system_model.h"
#include <vector>
#include "../../Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct Measurement {
    VectorXd value;
    std::string sensor_name;
};

class KalmanFilter {
    public:
        KalmanFilter(VectorXd x0, MatrixXd P0);
        VectorXd GetResidual();
        void AddModels(std::vector<SystemModel*> models);
        void EkfUpdate(std::vector<SystemModel*> sensors); 
        VectorXd GetState();
    private: 
        VectorXd _x;
        MatrixXd _P;
        VectorXd _r;
        std::vector<SystemModel *> _models; 
        VectorXd GetMeasurementVector(std::vector<SystemModel*> sensors);
        VectorXd GetPredictedMeasurementVector(std::vector<SystemModel*> sensors);
        MatrixXd GetSystemJacobian(std::vector<SystemModel*> systems, int m, double h);
        MatrixXd GetSystemVariance(std::vector<SystemModel*> systems, int m);
        void UpdateState();
};
#endif