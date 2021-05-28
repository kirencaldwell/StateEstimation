#include "sensor.h"

Sensor::Sensor() {}

Measurement Sensor::GetMeasurement(std::vector<State> x) {
  ComputeMeasurement(x);
  return _measurement;
}

MatrixXd Sensor::GetMeasurementMatrix(std::vector<State> x, double h) {
  ComputeMeasurementMatrix(x, h);
  return _measurement_matrix;
}

void Sensor::ComputeMeasurementMatrix(std::vector<State> x, double h) {
  // Get measurement for given states
  VectorXd y0 = GetMeasurement(x).GetValue();
  VectorXd yi = y0;
  // Compute size of measurement matrix
  int n = 0;
  for (int i = 0; i < (int)x.size(); i++) {
    n = n + x[i].GetValue().size();
  }
  MatrixXd H(y0.size(), n);

  // Fill in measurement matrix
  std::vector<State> x0 = x;
  int ctr = 0;
  for (int i = 0; i < (int)x.size(); i++) {
    VectorXd xi = x[i].GetValue();
    for (int j = 0; j < xi.size(); j++) {
      xi = GetIncrementedVector(xi, j, h);
      x[i].SetValue(xi);
      yi = GetMeasurement(x).GetValue(); 
      H.col(ctr) = (yi-y0)/h;
      xi = GetIncrementedVector(xi, j, -h);
      x[i].SetValue(xi);
      ctr++;
    }
  }
  _measurement_matrix = H;
}

VectorXd Sensor::GetIncrementedVector(VectorXd x, int i, double h) {
  x[i] = x[i] + h;
  return x;
}

void Sensor::UpdateState(std::vector<State> x) {
  for (int i = 0; i < (int)x.size(); i++) {
    for (int j = 0; j < (int)_states.size(); j++) {
      if (x[i].GetName() == _states[j].GetName()) {
        _states[j].SetValue(x[i].GetValue());
      }
    }
  }
}

void Sensor::ComputeMeasurement(std::vector<State> x) {
}
