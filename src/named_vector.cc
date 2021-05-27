#include "named_vector.h"

NamedVector::NamedVector(VectorXd initial_value, std::string name) {
  _value = initial_value;
  _name = name;
}

NamedVector::NamedVector() {}

VectorXd NamedVector::GetValue() {
  return _value;
}

std::string NamedVector::GetName() {
  return _name;
}

void NamedVector::SetName(std::string name) {
  _name = name;
}

void NamedVector::SetValue(VectorXd value) {
  _value = value;
}

NamedVector NamedVector::Cat(NamedVector a) {
  NamedVector out;
  VectorXd vec_joined(_value.size() + a.GetValue().size());
  vec_joined << _value, a.GetValue();
  out.SetValue(vec_joined);
  out.SetName(_name + " & " + a.GetName());
  return out;
}
