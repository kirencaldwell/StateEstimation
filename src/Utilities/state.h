#ifndef STATE_H_
#define STATE_H_

#include "../../Eigen/Dense"
#include <iostream>

template <class T>
class State {
    public:
        State(std::string state_name, T initial_value);
        T GetValue();
        T SetValue(T value);
        std::string GetName();
    protected:
        T _x;
        std::string _state_name;
};

#endif