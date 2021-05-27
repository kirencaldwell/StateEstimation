#ifndef NAMED_VECTOR_H
#define NAMED_VECTOR_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

using Eigen::VectorXd;
using namespace std;

class NamedVector {
  public: 
    NamedVector(VectorXd initial_value, std::string name);
    NamedVector();
    VectorXd GetValue();
    std::string GetName();
    void SetName(std::string name);
    void SetValue(VectorXd value);
    NamedVector Cat(NamedVector a);

  protected:
    VectorXd _value;
    std::string _name;
}; 

#endif
