#ifndef MEASUREMENT_H 
#define MEASUREMENT_H 

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#include "named_vector.h"

using Eigen::VectorXd;
using namespace std;

class Measurement : public NamedVector {
  public:
    Measurement(VectorXd value, std::string name);
    Measurement();
    Measurement Cat(Measurement a);
};

#endif 
