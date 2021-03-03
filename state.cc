#include "state.h"

State::State(std::string state_name, VectorXd initial_value) {
    _x = initial_value;
    _state_name = state_name;
}

VectorXd State::GetValue() {
    return _x;
}

void State::SetValue(VectorXd value) {
    _x = value;
}

std::string State::GetName() {
    return _state_name;
}