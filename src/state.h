#ifndef STATE_H
#define STATE_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#include "named_vector.h"

using Eigen::VectorXd;
using namespace std;

class State : public NamedVector {
  public:
    State(VectorXd value, std::string name);
    State();
    State Cat(State a);
};

#endif 
