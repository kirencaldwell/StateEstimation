#ifndef STATE_H_
#define STATE_H_

#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;

class State {
    public:
        State(std::string state_name, VectorXd initial_value);
        VectorXd GetValue();
        void SetValue(VectorXd value);
        std::string GetName();
    protected:
        VectorXd _x;
        std::string _state_name;
};

#endif