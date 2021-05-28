#include "kalman_filter.h"

KalmanFilter::KalmanFilter(std::vector<DynamicsModel*> models, MatrixXd P0) {
  _models = models;
  _P = P;
}

std::vector<State> KalmanFilter::GetState() {
  
  std::vector<State> out;
  std::vector<State> model_states;
  for (int i = 0; i < (int)_models.size(); i++) {
    model_states = _models->GetState();
   for (int j = 0; j < (int)model_states.size(); j++) {
     out.push_back(model_states[j]);
   }
  }
 return out; 
}

void KalmanFilter::UpdateFilter(double dt) {
}

void KalmanFilter::ApplyMeasurements(
    std::vector<Measurement> measurements, std::vector<Sensor*> sensors) {

}

void KalmanFilter::UpdateStates(double dt) {

}

MatrixXd GetStateTransitionMatrix(double dt, double h) {
  return MatrixXd::Identity(3,3);
}

MatrixXd GetMeasurementMatrix(double h) {
  return MatrixXd::Identity(3,3);
}
