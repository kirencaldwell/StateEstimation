#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "state.h"
#include "dynamics_model.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

std::string state_name = "position";
class TestModel : public DynamicsModel{
  public:
    TestModel(std::vector<State> x) : DynamicsModel(x) { 
    }
  private:
    std::vector<State> RunModel(std::vector<State> x) override {
      State position(VectorXd::Zero(3), "position");
      State velocity(VectorXd::Zero(3), "velocity");
      for (int i = 0; i < (int)x.size(); i++) {
        if (x[i].GetName() == "position") {    
          position.SetValue(x[i].GetValue());
        }
        if (x[i].GetName() == "velocity") {
          velocity.SetValue(x[i].GetValue());
        }
      } 
      
      std::vector<State> x_dot = x;
      for (int i = 0; i < (int)x.size(); i++) {
        if (x[i].GetName() == "position") {    
          x_dot[i].SetValue(velocity.GetValue());
        }
        if (x[i].GetName() == "velocity") {
          x_dot[i].SetValue(-5*position.GetValue() - 1*velocity.GetValue());
        }
      } 
      return x_dot;
    }
};

State position(VectorXd::Ones(3), "position");
State velocity(VectorXd::Zero(3), "velocity");
std::vector<State> states = {position, velocity};

TestModel test_model(states);

TEST(DynamicsModelTest, GetState) {
  std::vector<State> x = test_model.GetState();

  for (int i = 0; i < (int)x.size(); i++) {
    EXPECT_EQ(x[i].GetValue(), states[i].GetValue());
  }
}

TEST(DynamicsModelTest, StateTransitionMatrix) {
  double dt = 0.1;
  double h = 0.1;
  
  MatrixXd F_exp(6,6);
  F_exp << 1.0*MatrixXd::Identity(3,3), 0.1*MatrixXd::Identity(3,3), 
          -0.5*MatrixXd::Identity(3,3), 0.9*MatrixXd::Identity(3,3);
  MatrixXd F = test_model.GetStateTransitionMatrix(dt, h);
  ASSERT_TRUE(F.isApprox(F_exp));
}

TEST(DynamicsModelTest, UpdateState) {
  double dt = 0.1;
  VectorXd pos_exp(3);
  VectorXd vel_exp(3);
  pos_exp = position.GetValue() + dt*velocity.GetValue();
  vel_exp = velocity.GetValue() + dt*(-5*position.GetValue() - 1*velocity.GetValue());

  test_model.UpdateState(dt);
  std::vector<State> x_out = test_model.GetState();
  
  EXPECT_EQ(x_out[0].GetValue(), pos_exp);
  EXPECT_EQ(x_out[1].GetValue(), vel_exp);
}

