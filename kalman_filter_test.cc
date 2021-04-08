#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include <math.h>

#include "gps.h"
#include "translational_model.h"
#include "uncalibrated_accelerometer.h"
#include "attitude_deviation_model.h"
#include "bias_model.h"
#include "attitude_sensor_model.h"
#include "accelerometer.h"
#include "magnetometer.h"
#include "gyroscope.h"
#include "utilities.h"
#include "kalman_filter.h"
#include "system_model.h"
#include "telemetry_logging.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


int main(int argc, char *argv[]) {
    // Time information
    std::cout << "Initializing Time Information \n";
    double dt = 0.01;
    double Tf = 10;
    double t = 0;
    int k = 0;
    if (argc == 2) {
        Tf = std::atof(argv[1]);
    }

    TelemetryLogging tlm("logfile.csv");
    tlm.AddSignal("t", &t);

    // Oscillation information
    VectorXd f(3);
    f << 1, 2, 3;
    f = f/20;
    VectorXd phi(3);
    phi << 0, 45, 90;

    std::cout << "Initializing true states \n";
    // Sensor Variances
    MatrixXd R_accel = 0.01*MatrixXd::Identity(3, 3);
    MatrixXd R_magn = 0.001*MatrixXd::Identity(3, 3);
    MatrixXd R_gyro = 0.0001*MatrixXd::Identity(3, 3);
    MatrixXd R_eta = 0.5*MatrixXd::Identity(3, 3);
    MatrixXd R_gyro_bias = 0.1*MatrixXd::Identity(3, 3);
    MatrixXd R_accel_bias = 0.5*MatrixXd::Identity(3, 3);
    MatrixXd R_position = 0.001*MatrixXd::Identity(6, 6);
    MatrixXd R_gps = 1*MatrixXd::Identity(3, 3);

    // Setup true systems
    // true states
    VectorXd b_g = GetMvnRnd(R_gyro_bias);
    // VectorXd b_g = VectorXd::Zero(3);
    tlm.AddSignal("b_g", &b_g);
    VectorXd b_a = GetMvnRnd(R_accel_bias);
    tlm.AddSignal("b_a", &b_a);
    VectorXd x_attitude(6);
    x_attitude << 0, 0, 0, b_g;
    VectorXd x_nn = VectorXd::Zero(3);
    tlm.AddSignal("x_nn", &x_nn);
    VectorXd v_nn = VectorXd::Zero(3);
    tlm.AddSignal("v_nn", &v_nn);
    VectorXd x_translation(9);
    x_translation << x_nn, v_nn, b_a;
    MatrixXd R_bn = RotateVector(GetMvnRnd(R_eta));
    tlm.AddSignal("R_bn", &R_bn);
    VectorXd a_nn = VectorXd::Zero(3);
    tlm.AddSignal("a_nn", &a_nn);
    VectorXd w_bn = VectorXd::Zero(3);
    tlm.AddSignal("w_bn", &w_bn);
    
    std::cout << "Initializing true sensors \n"; 
    // Setup True Sensor Models
    Accelerometer accel_truth = Accelerometer();
    accel_truth.InitAccelerometer(&R_bn, &a_nn, &w_bn, &b_a, &dt);
    accel_truth.SetModelName("accel");
    accel_truth.SetVariance(R_accel);
    Magnetometer magn_truth = Magnetometer();
    magn_truth.InitMagnetometer(&R_bn, &w_bn, &dt);
    magn_truth.SetModelName("magn"); 
    magn_truth.SetVariance(R_magn);
    Gyroscope gyro_truth = Gyroscope();
    gyro_truth.InitGyroscope(&w_bn, &b_g);
    gyro_truth.SetModelName("gyro");
    gyro_truth.SetVariance(R_gyro);
    Gps gps_truth = Gps();
    gps_truth.SetModelName("gps");
    gps_truth.SetVariance(R_gps);
    VectorXd y_accel(3), y_magn(3), y_gyro(3), y_gps(3);
    tlm.AddSignal("accel", &y_accel);
    tlm.AddSignal("magn", &y_magn);
    tlm.AddSignal("gyro", &y_gyro);
    tlm.AddSignal("gps", &y_gps);
    
    std::cout << "Initializing estimator\n";
    // Setup estimator
    // estimated states
    VectorXd x_hat_attitude = VectorXd::Zero(6);
    tlm.AddSignal("x_hat_attitude", &x_hat_attitude);
    VectorXd b_g_hat = VectorXd::Zero(3);
    tlm.AddSignal("b_g_hat", &b_g_hat);
    VectorXd b_a_hat = VectorXd::Zero(3);
    tlm.AddSignal("b_a_hat", &b_a_hat);
    MatrixXd R_bn_hat = MatrixXd::Identity(3, 3);
    tlm.AddSignal("R_bn_hat", &R_bn_hat);
    VectorXd a_nn_hat = VectorXd::Zero(3);
    tlm.AddSignal("a_nn_hat", &a_nn_hat);
    VectorXd w_bn_hat = VectorXd::Zero(3);
    tlm.AddSignal("w_bn_hat", &w_bn_hat);
    VectorXd x_nn_hat = VectorXd::Zero(3);
    tlm.AddSignal("x_nn_hat", &x_nn_hat);
    VectorXd v_nn_hat = VectorXd::Zero(3);
    tlm.AddSignal("v_nn_hat", &v_nn_hat);
    VectorXd x_hat_translation = VectorXd::Zero(9);
    x_hat_translation << x_nn_hat, v_nn_hat, b_a_hat;
    // state error covariance
    MatrixXd P_attitude = 0.5*MatrixXd::Identity(x_hat_attitude.size(), x_hat_attitude.size());
    tlm.AddSignal("P_attitude", &P_attitude);
    MatrixXd P_translation = 0.5*MatrixXd::Identity(x_hat_translation.size(), x_hat_translation.size());
    tlm.AddSignal("P_translation", &P_translation);

    std::cout << "Initializing sensor models\n";
    // system sensor models
    Accelerometer accel_model = Accelerometer();
    accel_model.InitAccelerometer(&R_bn_hat, &a_nn_hat, &w_bn_hat, &b_a_hat, &dt);    
    accel_model.SetModelName("accel");
    accel_model.SetVariance(R_accel);
    Magnetometer magn_model = Magnetometer();
    magn_model.InitMagnetometer(&R_bn_hat, &w_bn_hat, &dt);
    magn_model.SetModelName("magn");
    magn_model.SetVariance(R_magn);
    Gyroscope gyro_model = Gyroscope();
    gyro_model.InitGyroscope(&y_gyro, &b_g_hat);
    gyro_model.SetModelName("gyro"); 
    gyro_model.SetVariance(R_gyro);
    UncalibratedAccelerometer uncal_accel_model = UncalibratedAccelerometer();
    uncal_accel_model.InitUncalibratedAccelerometer(&R_bn_hat, &a_nn_hat, &w_bn_hat, &R_bn_hat); 
    uncal_accel_model.SetModelName("accel");
    uncal_accel_model.SetVariance(R_accel);
    Gps gps_model = Gps();
    gps_model.SetModelName("gps");
    gps_model.SetVariance(R_gps);
    accel_model.AddMeasurement(accel_truth.GetNoisyOutput(x_attitude));
    magn_model.AddMeasurement(magn_truth.GetNoisyOutput(x_attitude));
    uncal_accel_model.AddMeasurement(accel_truth.GetNoisyOutput(x_attitude));
    gps_model.AddMeasurement(gps_truth.GetNoisyOutput(x_translation));

    std::cout << "Initializing update models\n";
    // system update model
    AttitudeSensorModel attitude_model = AttitudeSensorModel();
    attitude_model.InitAttitudeSensorModel(&R_bn_hat, &y_gyro, &dt);     
    attitude_model.SetModelName("attitude");
    BiasModel gyro_bias_model = BiasModel(); 
    gyro_bias_model.InitBiasModel();
    gyro_bias_model.SetModelName("gyro_bias");  
    gyro_bias_model.SetVariance(0.00001*R_gyro_bias); 

    AttitudeDeviationModel deviation_model = AttitudeDeviationModel();
    deviation_model.InitAttitudeDeviationModel(&R_bn_hat, &y_gyro, &b_g_hat, &dt);
    deviation_model.SetModelName("deviation");
    deviation_model.SetVariance(R_gyro);

    TranslationalModel translation_model = TranslationalModel();
    translation_model.InitTranslationalModel(&y_accel, &R_bn_hat, &dt);
    translation_model.SetModelName("translation");
    translation_model.SetVariance(R_position); 
    BiasModel accel_bias_model = BiasModel();
    accel_bias_model.InitBiasModel();
    accel_bias_model.SetModelName("accel_bias");
    accel_bias_model.SetVariance(0.001*R_accel_bias);

    std::vector<SystemModel*> attitude_process_models;
    attitude_process_models.push_back(&deviation_model);
    attitude_process_models.push_back(&gyro_bias_model);
    std::vector<SystemModel*> attitude_sensor_models;
    
    std::vector<SystemModel*> translation_process_models;
    translation_process_models.push_back(&translation_model);
    translation_process_models.push_back(&accel_bias_model);
    std::vector<SystemModel*> translation_sensor_models;

    std::cout << "Initializing Kalman Filter\n";
    // Initialize Kalman Filters
    KalmanFilter attitude_kf(x_hat_attitude, P_attitude);
    attitude_kf.AddModels(attitude_process_models);
    KalmanFilter translation_kf(x_hat_translation, P_translation);
    translation_kf.AddModels(translation_process_models);

    std::cout << "Adding Telemetry Signals\n";
    // ======= Simulate ==========
    tlm.CreateLogHeader();
    std::cout << "Starting Simulation...\n";
    while (t < Tf) {
        
        PrintTimeUpdate(t, R_bn_hat, int(1/dt));

        // std::cout << "Take measurements\n\n";
        y_accel = accel_truth.GetNoisyOutput(x_attitude);
        y_magn = magn_truth.GetNoisyOutput(x_attitude);
        y_gyro = gyro_truth.GetNoisyOutput(x_attitude);
        y_gps = gps_truth.GetNoisyOutput(x_translation);
        accel_model.AddMeasurement(y_accel);
        magn_model.AddMeasurement(y_magn);
        gyro_model.AddMeasurement(y_gyro);
        uncal_accel_model.AddMeasurement(y_accel);
        gps_model.AddMeasurement(y_gps);
        attitude_sensor_models = {&accel_model, &magn_model};
        translation_sensor_models = {&gps_model, &uncal_accel_model};

        // Update linearization point
        w_bn_hat = gyro_model.GetOutput(x_hat_attitude);
        b_g_hat = -1*GetSubspace(x_hat_attitude, 3, 5);
        attitude_model.RunModel(x_hat_attitude);
        // std::cout << "Estimate attitude states \n\n";
        attitude_kf.EkfUpdate(attitude_sensor_models);
        x_hat_attitude = attitude_kf.GetState();
        // Apply correction
        attitude_model.ApplyAttitudeCorrection(x_hat_attitude); 
        // std::cout << "R_bn_hat = " << R_bn_hat << "\n\n";

        // a_nn_hat = uncal_accel_model.GetOutput(x_hat_translation);
        // std::cout << "Estimate translational states \n\n";
        translation_kf.EkfUpdate(translation_sensor_models);
        x_nn_hat = translation_kf.GetState().head(3);
        v_nn_hat = GetSubspace(translation_kf.GetState(), 3, 5);
        b_a_hat = 1*translation_kf.GetState().tail(3);
        // std::cout << "accel residual = \n";
        // std::cout << accel_cal_kf.GetResidual() << "\n\n";

        // Logging
        tlm.LogSignals();
        
        // update true states
        VectorXd ww = 2*M_PI*t*f + phi*M_PI/180;
        w_bn = ww.array().cos();
        R_bn = R_bn*RotateVector(dt*w_bn);
        VectorXd aa = 2*M_PI*t*f/5;
        a_nn = aa.array().sin();
        x_nn = x_nn + dt*v_nn;
        v_nn = v_nn + dt*a_nn;
        x_translation << x_nn, v_nn, b_a;
        t += dt;
        k++;
    
    }
    std::cout << "gyro bias error = " << (b_g_hat-b_g).norm()/(b_g.norm()) << "\n";
    std::cout << "accel bias error = " << (b_a_hat-b_a).norm()/(b_a.norm()) << "\n\n";
    tlm.EndLogging();
    std::cout << "\nFinished\n";
    return 0;
}
