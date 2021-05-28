#include "named_vector.h"

NamedVectorArray::NamedVectorArray(NamedVector x0) {
  _value.push_back(x0);
}

NamedVectorArray::NamedVectorArray(std::vector<NamedVector> x0) {
  _value = x0;
}

NamedVectorArray::NamedVectorArray() {}

VectorXd NamedVectorArray::AsVector() {
  VectorXd out;
  for (int i = 0; i < (int)_value.size(); i++) {
    VectorXd temp(out.size() + _value[i].value.size());
    temp << out, _value[i].value;
    out = temp;
  }
  return out;
}

VectorXd NamedVectorArray::GetVectorByName(std::string name) {
  for (int i = 0; i < (int)_value.size(); i++) {
    if (name == _value[i].name) {
      return _value[i].value;
    }
  }
  return VectorXd(0);
}

