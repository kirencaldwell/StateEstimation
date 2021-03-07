#ifndef SYSTEM_MODEL_H_
#define SYSTEM_MODEL_H_

#include <iostream>
#include "Eigen/Dense"
#include "utilities.h"
#include "state.h"
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;

// struct Measurement {
//     std::string sensor_name;
//     VectorXd value;
// };

class SystemModel {
    public:
        void AddMeasurement(VectorXd y); 
        VectorXd GetMeasurement();
        void ClearMeasurement();
        void SetModelName(std::string model_name);
        void SetVariance(MatrixXd R);
        virtual MatrixXd GetJacobian(VectorXd x, double h);
        VectorXd GetOutput(VectorXd x);
        std::string GetName();
        
        virtual VectorXd GetNoisyOutput(VectorXd x);
        virtual VectorXd RunModel(VectorXd x);
        // virtual VectorXd RunModel(std::vector<State<VectorXd>> x);
        virtual MatrixXd GetVariance();
        virtual MatrixXd GetVariance(VectorXd x);
    protected:
        std::string _name;
        MatrixXd _R;
        VectorXd _y;
};

#endif