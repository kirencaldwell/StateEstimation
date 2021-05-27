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
State position(VectorXd::Ones(3), sensor_name);
State velocity(VectorXd::Zero(3), "velocity");
std::vector<State> states = {position, velocity};

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

TEST(SensorTest, GetMeasurement) {
  TestSensor test_sensor({position});
  Measurement y = test_sensor.GetMeasurement(states);
  EXPECT_EQ(y.GetValue(), 5*VectorXd::Ones(3)); 

}
