#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <vector>
#include "measurement.h"
#include "state.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter {
  public:
    KalmanFilter(std::vector<DynamicsModel*> models, MatrixXd P0);
    std::vector<State> GetState();
    void UpdateFilter(double dt);
    void ApplyMeasurements(std::vector<Measurement> measurements, std::vector<Sensor*> sensors);

  private:
    MatrixXd _P;
    std::vector<DynamicsModel*> _models;

    void UpdateStates(double dt);
    MatrixXd GetStateTransitionMatrix(double dt, double h);
    MatrixXd GetMeasurementMatrix(double h);
    
#endif
