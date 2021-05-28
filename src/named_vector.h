#ifndef NAMED_VECTOR_H
#define NAMED_VECTOR_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

using Eigen::VectorXd;

struct NamedVector {
  VectorXd value;
  std::string name;
};

class NamedVectorArray {
  public:
    NamedVectorArray(NamedVector x0);
    NamedVectorArray(std::vector<NamedVector> x0); 
    NamedVectorArray();

    VectorXd AsVector();
    VectorXd GetVectorByName(std::string name);
      
  protected:
    std::vector<NamedVector> _value;
}; 

#endif
