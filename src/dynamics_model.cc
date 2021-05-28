#include "dynamics_model.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

DynamicsModel::DynamicsModel(std::vector<State> x0) {
  _states = x0;
}

void DynamicsModel::UpdateState(double dt) {
  _states = ComputeNextState(_states, dt);
}

std::vector<State> DynamicsModel::ComputeNextState(std::vector<State> x, double dt) {
  std::vector<State> x_dot = RunModel(x);
  for (int i = 0; i < (int)_states.size(); i++) {
    x[i].SetValue(x[i].GetValue() + dt*x_dot[i].GetValue());
  }
  return x;
}

std::vector<State> DynamicsModel::GetState() {
  return _states;
}

std::vector<State> DynamicsModel::RunModel(std::vector<State> x) {
  return _states;
}

MatrixXd DynamicsModel::GetStateTransitionMatrix(double dt, double h){
  return ComputeStateTransitionMatrix(dt, h);
}

MatrixXd DynamicsModel::ComputeStateTransitionMatrix(double dt, double h) {
  std::vector<State> xx = _states;
  
  int n = 0;
  for (int i = 0; i < (int)_states.size(); i++) {
    n = n + _states[i].GetValue().size();
  }
  MatrixXd F(n, n);
  
  int ctr = 0;
  for (int i = 0; i < (int)_states.size(); i++) {
    MatrixXd Fi = GetProcessTransitionMatrix(dt, h, i, n);
    F.block(ctr,0,Fi.rows(),n) << Fi;
    ctr = ctr + Fi.rows();
  }
  return F;

}

MatrixXd DynamicsModel::GetProcessTransitionMatrix(double dt, double h, int idx, int n_states) {
  int n = _states[idx].GetValue().size();
  MatrixXd F(n, n_states);

  std::vector<State> next_states = ComputeNextState(_states, dt);  
  VectorXd y0 = next_states[idx].GetValue();
  std::vector<State> x = _states;
  VectorXd yi = y0;
  int ctr = 0;
  for (int i = 0; i < (int)x.size(); i++) {
    VectorXd xi = x[i].GetValue();
    for (int j = 0; j < xi.size(); j++) {
      xi = GetIncrementedVector(xi, j, h);
      x[i].SetValue(xi);
      next_states = ComputeNextState(x, dt);  
      yi = next_states[idx].GetValue();
      F.col(ctr) = (yi-y0)/h;
      xi = GetIncrementedVector(xi, j, -h);
      x[i].SetValue(xi);
      ctr++;
    }
  }
  return F;
}

VectorXd DynamicsModel::GetIncrementedVector(VectorXd x, int i, double h) {
  x[i] = x[i] + h;
  return x;
}
