#include "generic_sensor.h"

void GenericSensor::SetSensorName(std::string sensor_name) {
    _name = sensor_name;
}

void GenericSensor::SetVariance(MatrixXd R) {
    _R = R;
}

VectorXd GenericSensor::GetNoisyMeasurement(VectorXd x) {
    VectorXd v(3);
    v(0) = 0.1;
    v(1) = -0.1;
    v(2) = 0.5;
    return MeasurementFunction(x) + v; 
}

VectorXd GenericSensor::GetPredictedMeasurement(VectorXd x) {
    return MeasurementFunction(x);
}

VectorXd GenericSensor::MeasurementFunction(VectorXd x) {
    return x;
}

MatrixXd GenericSensor::GetJacobian(VectorXd x, double h){
    VectorXd y0 = MeasurementFunction(x);
    int m = y0.size();
    int n = x.size();
    MatrixXd H(m, n);

    for (int i = 0; i < n; i++) {
        VectorXd x0 = x;
        x0(i) = x0(i) + h;
        H.col(i) = (MeasurementFunction(x0) - y0) / h;
    } 
     
    return H;
}

std::string GenericSensor::GetName() {
    return _name;
}
