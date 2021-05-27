#include "state.h"

State::State(VectorXd value, std::string name) {
  _value = value;
  _name = name;
}

State::State() {}

State State::Cat(State a) {
  State out;
  VectorXd vec_joined(_value.size() + a.GetValue().size());
  vec_joined << _value, a.GetValue();
  out.SetValue(vec_joined);
  out.SetName(_name + " & " + a.GetName());
  return out;
}
