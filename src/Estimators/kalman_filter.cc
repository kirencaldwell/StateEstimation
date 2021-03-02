#include "kalman_filter.h"


KalmanFilter::KalmanFilter(VectorXd x0, MatrixXd P0) {
    _x = x0;
    _P = P0;
    std::cout << "Initial State: \n" << _x << std::endl;    
    std::cout << "Initial Covariance: \n" << _P << std::endl;
}

void KalmanFilter::AddModels(std::vector<SystemModel *> models) {
    _models = models;
}

// void KalmanFilter::AddSensors(std::vector<SystemModel *> sensors) {
//     _sensors = sensors;
// }

MatrixXd KalmanFilter::GetSystemJacobian(std::vector<SystemModel*> systems, int m, double h) {
    MatrixXd H(m, _x.size());
    int ctr = 0;
    for (int i = 0; i < systems.size(); i++) {
        MatrixXd H_i = systems[i]->GetJacobian(_x, h);
        for (int j = 0; j < H_i.rows(); j++) {
            H.row(ctr) = H_i.row(j);
            ctr = ctr+1;
        }
    }
    return H;
}

// MatrixXd KalmanFilter::GetSystemJacobian(std::vector<Measurement*> measurements, int m, double h) {
//     MatrixXd H(m, _x.size());
//     int ctr = 0;
//     for (int i = 0; i < measurements.size(); i++) {
//         MatrixXd H_i = measurements[i]->model.GetJacobian(_x, h);
//         for (int j = 0; j < H_i.rows(); j++) {
//             H.row(ctr) = H_i.row(j);
//             ctr = ctr+1;
//         }
//     }
//     return H;
// }

// MatrixXd KalmanFilter::GetSystemVariance(std::vector<Measurement*> measurements, int m) {
//     MatrixXd R = MatrixXd::Zero(m, m);
//     int idx = 0;
//     int j;
//     for (int i = 0; i < measurements.size(); i++) {
//         MatrixXd R_i = measurements[i]->model.GetVariance();
//         for (j = 0; j < R_i.cols(); j++) {
//             for (int k = 0; k < R_i.rows(); k++) {
//                 R(idx+j, idx+k) = R_i(j, k);
//             }
//         }
//     }
//     return R;
// }

MatrixXd KalmanFilter::GetSystemVariance(std::vector<SystemModel*> systems, int m) {
    MatrixXd R = MatrixXd::Zero(m, m);
    int idx = 0;
    int j;
    for (int i = 0; i < systems.size(); i++) {
        MatrixXd R_i = systems[i]->GetVariance();
        for (j = 0; j < R_i.cols(); j++) {
            for (int k = 0; k < R_i.rows(); k++) {
                R(idx+j,idx+k) = R_i(j,k);
            }
        }
        idx += j;
    }
    return R;
}

void KalmanFilter::UpdateState() {
    int idx = 0;
    int j;
    for (int i = 0; i < _models.size(); i++) {
        VectorXd x_i = _models[i]->RunModel(_x);
        for (j = 0; j < x_i.size(); j++) {
            _x(idx+j) = x_i(j);
        }
        idx = j;
    }
}

VectorXd KalmanFilter::GetResidual() {
    return _r;
}

void KalmanFilter::EkfUpdate(std::vector<SystemModel*> sensors) {
    
    UpdateState();
    // std::cout << "x = " << _x << "\n\n";

    MatrixXd F = GetSystemJacobian(_models, _x.size(), 0.0001);
    // std::cout << "F = " << F << "\n\n";

    MatrixXd Q = GetSystemVariance(_models, _x.size());
    // std::cout << "Q = " << Q << "\n\n";
    _P = F*_P*F.transpose() + Q;
    // std::cout << "P = " << _P << "\n\n";
    
    if (sensors.size() > 0) {
        VectorXd y = GetMeasurementVector(sensors);
        // std::cout << " y = " << y << "\n\n";
        VectorXd z = GetPredictedMeasurementVector(sensors);
        // std::cout << "z = " << z << "\n\n";
        // std::cout << "r = " << y-z << "\n\n";

        MatrixXd H = GetSystemJacobian(sensors, y.size(), 0.001);
        // std::cout << "H = " << H << "\n\n";
        MatrixXd R = GetSystemVariance(sensors, y.size());
        // std::cout << "R = " << R << "\n\n";

        MatrixXd S = H*_P*H.transpose() + R;
        
        MatrixXd K = _P*H.transpose()*S.inverse();
        // std::cout << "K = " << K << "\n\n";
        _x = _x + K*(y-z);
        _P = (MatrixXd::Identity(_x.size(), _x.size()) - K*H)*_P;

        z = GetPredictedMeasurementVector(sensors);
        _r = y-z;
    }
    
}

VectorXd KalmanFilter::GetState() {
    return _x;
}

VectorXd KalmanFilter::GetPredictedMeasurementVector(std::vector<SystemModel*> sensors) {
    VectorXd z = VectorXd::Zero(0);
    for (int i = 0; i < sensors.size(); i++) {
        VectorXd zi = sensors[i]->GetOutput(_x);
        z.conservativeResize(z.size()+zi.size(),1);
        z.tail(zi.size()) = zi;
    }
    
    return z;    
}

VectorXd KalmanFilter::GetMeasurementVector(std::vector<SystemModel*> sensors) {
    VectorXd y = VectorXd::Zero(0);
    for (int i = 0; i < sensors.size(); i++) {
        VectorXd yi = sensors[i]->GetMeasurement();
        y.conservativeResize(y.size()+yi.size(),1);
        y.tail(yi.size()) = yi;
    }
    return y;
}