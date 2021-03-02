#include "system_model.h"

void SystemModel::SetModelName(std::string model_name) {
    _name = model_name;
    std::cout << "Setting model name: " << _name << "\n";
}

void SystemModel::AddMeasurement(VectorXd y) {
    _y = y;
    // std::cout << "yii = " << _y << "\n\n";
}

VectorXd SystemModel::GetMeasurement() {
    return _y;
}

void SystemModel::ClearMeasurement() {
    _y = VectorXd::Zero(0);
}

void SystemModel::SetVariance(MatrixXd R) {
    _R = R;
}

MatrixXd SystemModel::GetVariance() {
    return _R;
}

MatrixXd SystemModel::GetVariance(VectorXd x) {
    return _R;
}

std::string SystemModel::GetName() {
    return _name;
}

VectorXd SystemModel::GetOutput(VectorXd x){
    return RunModel(x);
}

VectorXd SystemModel::RunModel(VectorXd x) {
    return x;
}

VectorXd SystemModel::GetNoisyOutput(VectorXd x) {
    normal_random_variable v { _R };
    VectorXd y = RunModel(x) + v();
    return y;
}

MatrixXd SystemModel::GetJacobian(VectorXd x, double h){
    VectorXd y0 = RunModel(x);
    int m = y0.size();
    int n = x.size();
    MatrixXd H(m, n);
    for (int i = 0; i < n; i++) {
        VectorXd x0 = x;
        x0(i) = x0(i) + h;
        H.col(i) = (RunModel(x0) - y0) / h;
    } 
     
    return H;
}