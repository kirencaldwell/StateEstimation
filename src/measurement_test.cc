
#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "state.h"

using Eigen::VectorXd;

State x = State(VectorXd::Zero(3), "x");
State y = State(VectorXd::Ones(3), "y");

TEST(StateTest, TestGetName) {
  EXPECT_EQ(x.GetName(), "x");
}

TEST(StateTest, TestGetValue) {
  EXPECT_EQ(x.GetValue(), VectorXd::Zero(3));
}

TEST(StateTest, TestCat) {
  State z = x.Cat(y);
  VectorXd z_exp(6);
  z_exp << 0, 0, 0, 1, 1, 1;
  EXPECT_EQ(z.GetValue(), z_exp);
  EXPECT_EQ(z.GetName(), "x & y");
}
