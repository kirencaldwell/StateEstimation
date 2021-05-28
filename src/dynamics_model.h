#ifndef DYNAMICS_MODEL_H
#define DYNAMICS_MODEL_H

#include <vector>
#include "measurement.h"
#include "state.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class DynamicsModel {
  public: 
    DynamicsModel(std::vector<State> x0);
    void UpdateState(double dt);
    std::vector<State> GetState();
    MatrixXd GetStateTransitionMatrix(double dt, double h);

  protected:
    std::vector<State> _states;
    virtual std::vector<State> RunModel(std::vector<State> x); 
    std::vector<State> ComputeNextState(std::vector<State> x, double dt);
    MatrixXd ComputeStateTransitionMatrix(double dt, double h);
    MatrixXd GetProcessTransitionMatrix(double dt, double h, int idx, int n_states);
    VectorXd GetIncrementedVector(VectorXd x, int i, double h); 
};
#endif
