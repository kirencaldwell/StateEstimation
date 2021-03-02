#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include <math.h>

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
    Signal time("t");
    time.SetSignal(&t);

    // Oscillation information
    VectorXd f(3);
    f << 1, 2, 3;
    f = f/20;
    VectorXd phi(3);
    phi << 0, 45, 90;

    std::cout << "Initializing true states \n";
    // Sensor Variances
    MatrixXd R_accel = 0.1*MatrixXd::Identity(3, 3);
    MatrixXd R_magn = 0.01*MatrixXd::Identity(3, 3);
    MatrixXd R_gyro = 0.0001*MatrixXd::Identity(3, 3);
    MatrixXd R_eta = 0.5*MatrixXd::Identity(3, 3);
    MatrixXd R_gyro_bias = 0.1*MatrixXd::Identity(3, 3);
    MatrixXd R_accel_bias = 0.5*MatrixXd::Identity(3, 3);

    // Setup true systems
    // true states
    normal_random_variable b_g_v {R_gyro_bias};
    VectorXd b_g = b_g_v();
    // VectorXd b_g = VectorXd::Zero(3);
    Signal gyro_bias("b_g");
    gyro_bias.SetSignal(&b_g);
    normal_random_variable b_a_v {R_accel_bias};
    VectorXd b_a = b_a_v();
    Signal accel_bias("b_a");
    accel_bias.SetSignal(&b_a);
    VectorXd x_attitude(6);
    x_attitude << 0, 0, 0, b_g;
    normal_random_variable eta {R_eta};
    MatrixXd R_bn = RotateVector(eta());
    Signal true_attitude("R_bn");
    true_attitude.SetSignal(&R_bn);
    VectorXd w_bn = VectorXd::Zero(3);
    Signal true_w_bn("w_bn");
    true_w_bn.SetSignal(&w_bn);
    
    std::cout << "Initializing true sensors \n"; 
    // Setup True Sensor Models
    Accelerometer accel_truth = Accelerometer();
    accel_truth.InitAccelerometer(&R_bn, &w_bn, &b_a, &dt);
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
    VectorXd y_accel(3), y_magn(3), y_gyro(3);
    Signal accel_signal("accel"), magn_signal("magn"), gyro_signal("gyro");
    accel_signal.SetSignal(&y_accel); magn_signal.SetSignal(&y_magn); gyro_signal.SetSignal(&y_gyro); 
    
    std::cout << "Initializing estimator\n";
    // Setup estimator
    // estimated states
    VectorXd x_hat = VectorXd::Zero(6);
    Signal estimated_state("x_hat");
    estimated_state.SetSignal(&x_hat);
    VectorXd b_g_hat = VectorXd::Zero(3);
    Signal estimated_gyro_bias("b_g_hat");
    estimated_gyro_bias.SetSignal(&b_g_hat);
    VectorXd b_a_hat = VectorXd::Zero(3);
    Signal estimated_accel_bias("b_a_hat");
    estimated_accel_bias.SetSignal(&b_a_hat);
    MatrixXd R_bn_hat(3,3);
    R_bn_hat << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;
    Signal estimated_attitude("R_bn_hat");
    estimated_attitude.SetSignal(&R_bn_hat);
    VectorXd w_bn_hat(3);
    w_bn_hat << 0, 0, 0;
    Signal estimated_w_bn("w_bn_hat");
    estimated_w_bn.SetSignal(&w_bn_hat);
    // state covariance
    MatrixXd P = 0.5*MatrixXd::Identity(x_hat.size(), x_hat.size());
    Signal estimated_covariance("P_hat");
    estimated_covariance.SetSignal(&P);

    std::cout << "Initializing sensor models\n";
    // system sensor models
    Accelerometer accel_model = Accelerometer();
    accel_model.InitAccelerometer(&R_bn_hat, &w_bn_hat, &b_a_hat, &dt);    
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
    accel_model.AddMeasurement(accel_truth.GetNoisyOutput(x_attitude));
    magn_model.AddMeasurement(magn_truth.GetNoisyOutput(x_attitude));

    std::cout << "Initializing update models\n";
    // system update model
    AttitudeSensorModel attitude_model = AttitudeSensorModel();
    attitude_model.InitAttitudeSensorModel(&R_bn_hat, &y_gyro, &dt);     
    attitude_model.SetModelName("attitude");
    BiasModel gyro_bias_model = BiasModel(); 
    gyro_bias_model.InitBiasModel();
    gyro_bias_model.SetModelName("gyro_bias");  
    gyro_bias_model.SetVariance(0.00001*R_gyro_bias); 
    BiasModel accel_bias_model = BiasModel();
    accel_bias_model.SetModelName("accel_bias");
    accel_bias_model.SetVariance(0.00001*R_accel_bias);

    AttitudeDeviationModel deviation_model = AttitudeDeviationModel();
    deviation_model.InitAttitudeDeviationModel(&R_bn_hat, &y_gyro, &b_g_hat, &dt);
    deviation_model.SetModelName("deviation");
    deviation_model.SetVariance(R_gyro);

    std::vector<SystemModel*> models;
    models.push_back(&deviation_model);
    models.push_back(&gyro_bias_model);
    // models.push_back(&accel_bias_model);
    std::vector<SystemModel*> sensor_models;

    std::cout << "Initializing Kalman Filter\n";
    // Initialize Kalman Filter
    KalmanFilter kf(x_hat, P);
    kf.AddModels(models);

    std::cout << "Adding Telemetry Signals\n";
    // ======= Simulate ==========
    TelemetryLogging tlm("logfile.csv");
    tlm.AddSignal(time);
    tlm.AddSignal(true_attitude);
    tlm.AddSignal(true_w_bn);
    tlm.AddSignal(estimated_attitude);
    tlm.AddSignal(estimated_state);
    tlm.AddSignal(estimated_w_bn);
    tlm.AddSignal(estimated_gyro_bias);
    tlm.AddSignal(estimated_accel_bias);
    tlm.AddSignal(estimated_covariance);
    tlm.AddSignal(accel_signal);
    tlm.AddSignal(magn_signal);
    tlm.AddSignal(gyro_signal);
    tlm.AddSignal(gyro_bias);
    tlm.AddSignal(accel_bias);
    tlm.CreateLogHeader();
    std::cout << "Starting Simulation...\n";
    while (t < Tf) {
        
        PrintTimeUpdate(t, R_bn_hat, int(1/dt));

        // std::cout << "Take measurements\n\n";
        y_accel = accel_truth.GetNoisyOutput(x_attitude);
        y_magn = magn_truth.GetNoisyOutput(x_attitude);
        y_gyro = gyro_truth.GetNoisyOutput(x_attitude);
        accel_model.AddMeasurement(y_accel);
        magn_model.AddMeasurement(y_magn);
        gyro_model.AddMeasurement(y_gyro);
        sensor_models = {&accel_model, &magn_model};

        // Update linearization point
        w_bn_hat = gyro_model.GetOutput(x_hat);
        b_g_hat = -1*GetSubspace(x_hat, 3, 5);
        // b_a_hat = -1*GetSubspace(x_hat, 6, 8);
        attitude_model.RunModel(x_hat);
        // std::cout << "Run EKF\n\n";
        kf.EkfUpdate(sensor_models);
        x_hat = kf.GetState();
        // Apply correction
        attitude_model.ApplyAttitudeCorrection(x_hat); 
        // std::cout << "R_bn_hat = " << R_bn_hat << "\n\n";
        
        // Logging
        tlm.LogSignals();
        
        // update true states
        VectorXd ww = 2*M_PI*t*f + phi*M_PI/180;
        w_bn = ww.array().cos();
        R_bn = R_bn*RotateVector(dt*w_bn);
        t += dt;
        k++;
    
    }
    std::cout << "gyro bias = " << b_g_hat << "\n";

    tlm.EndLogging();
    std::cout << "\nFinished\n";
    return 0;
}
