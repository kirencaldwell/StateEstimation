#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "named_vector.h"

using Eigen::VectorXd;

VectorXd a = VectorXd::Ones(3);
VectorXd b = 2*VectorXd::Ones(3);
VectorXd c(a.size()+b.size());

NamedVector x = {.value = a,
                 .name = "position"};
NamedVector y = {.value = b,
                 .name = "velocity"};


NamedVectorArray vec1(x);
NamedVectorArray vec2({x, y});

TEST(NamedVectorTests, GetAsVector) {
  EXPECT_EQ(a, vec1.AsVector());

  c << a, b;
  EXPECT_EQ(c, vec2.AsVector());
}

TEST(NamedVectorTests, GetByName) {
  EXPECT_EQ(a, vec1.GetVectorByName("position"));
  EXPECT_EQ(b, vec2.GetVectorByName("velocity"));
}
