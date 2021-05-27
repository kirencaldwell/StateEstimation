#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "named_vector.h"

using Eigen::VectorXd;

NamedVector x = NamedVector(VectorXd::Zero(3), "x");
NamedVector y = NamedVector(VectorXd::Ones(3), "y");

TEST(NamedVectorTest, TestGetName) {
  EXPECT_EQ(x.GetName(), "x");
}

TEST(NamedVectorTest, TestGetValue) {
  EXPECT_EQ(x.GetValue(), VectorXd::Zero(3));
}

TEST(NamedVectorTest, TestCat) {
  NamedVector z = x.Cat(y);
  VectorXd z_exp(6);
  z_exp << 0, 0, 0, 1, 1, 1;
  EXPECT_EQ(z.GetValue(), z_exp);
  EXPECT_EQ(z.GetName(), "x & y");
}

  
