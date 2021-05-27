#include "measurement.h"

Measurement::Measurement(VectorXd value, std::string name) {
  _value = value;
  _name = name;
}

Measurement::Measurement() {}

Measurement Measurement::Cat(Measurement a) {
  Measurement out;
  VectorXd vec_joined(_value.size() + a.GetValue().size());
  vec_joined << _value, a.GetValue();
  out.SetValue(vec_joined);
  out.SetName(_name + " & " + a.GetName());
  return out;
}
