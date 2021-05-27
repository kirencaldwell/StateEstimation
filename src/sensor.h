#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include "measurement.h"
#include "state.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Sensor {
  public: 
    Sensor();
    Measurement GetMeasurement(std::vector<State> x);
    MatrixXd GetMeasurementMatrix(std::vector<State> x); 

  protected:
    MatrixXd _measurement_matrix;
    std::vector<State> _states;
    Measurement _measurement;

    void ComputeMeasurementMatrix();
    void UpdateState(std::vector<State> x);
    virtual void ComputeMeasurement(std::vector<State> x);
};

#endif