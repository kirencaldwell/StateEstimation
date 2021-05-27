#include "sensor.h"

Sensor::Sensor() {}

Measurement Sensor::GetMeasurement(std::vector<State> x) {
  ComputeMeasurement(x);
  return _measurement;
}

MatrixXd Sensor::GetMeasurementMatrix(std::vector<State> x) {
  return _measurement_matrix;
}

void Sensor::ComputeMeasurementMatrix() {
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
