#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "state.h"
#include "measurement.h"
#include "sensor.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

std::string sensor_name = "position";
class TestSensor : public Sensor {
  public:
    TestSensor(std::vector<State> x) {
      _states = x;
    }
  private:
    void ComputeMeasurement(std::vector<State> x) override {
      State position(VectorXd::Zero(3), sensor_name);
      for (int i = 0; i < (int)x.size(); i++) {
        if (x[i].GetName() == sensor_name) {    
          position.SetValue(x[i].GetValue());
        }
      } 
     _measurement.SetValue(5*position.GetValue());
    _measurement.SetName(sensor_name); 
    }
};

State position(VectorXd::Ones(3), sensor_name);
State velocity(VectorXd::Zero(3), "velocity");
std::vector<State> states = {position, velocity};

TestSensor test_sensor({position});


TEST(SensorTest, GetMeasurement) {
  Measurement y = test_sensor.GetMeasurement(states);
  EXPECT_EQ(y.GetValue(), 5*VectorXd::Ones(3));  

}

TEST(SensorTest, GetMeasurementMatrix) {
  double h = 0.1;
  VectorXd xi = test_sensor.GetIncrementedVector(states[1].GetValue(), 1, h);
  VectorXd xi_exp(3); 
  xi_exp << 0, h, 0;
  EXPECT_EQ(xi, xi_exp); 

  MatrixXd H = test_sensor.GetMeasurementMatrix(states, h);
  MatrixXd H_exp(3, 6);
  H_exp << 5*MatrixXd::Identity(3,3), MatrixXd::Zero(3,3);
  ASSERT_TRUE(H.isApprox(H_exp));
}
