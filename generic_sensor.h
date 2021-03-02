#ifndef GENERIC_SENSOR_H_
#define GENERIC_SENSOR_H_

#include "system_model.h"
#include <iostream>
#include "Eigen/Dense"
#include "utilities.h"
using Eigen::VectorXd;
using Eigen::MatrixXd;

class GenericSensor: public SystemModel {
    public:

        void SetSensorName(std::string sensor_name); 
        std::string GetName();
        VectorXd GetNoisyMeasurement(VectorXd x);        
        VectorXd GetPredictedMeasurement(VectorXd x);
        void SetVariance(MatrixXd R);

        virtual VectorXd MeasurementFunction(VectorXd x);
        MatrixXd GetJacobian(VectorXd x, double h);
    private:
        std::string _name;
        MatrixXd _R;
    protected:

};


#endif