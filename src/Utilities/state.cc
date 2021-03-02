#include "state.h"

template <class T>
State<T>::State(std::string state_name, T initial_value) {
    _x = initial_value;
    _state_name = state_name;
}

template <class T>
T State<T>::GetValue() {
    return _x;
}

template <class T>
T State<T>::SetValue(T value) {
    _x = value;
}

template <class T>
std::string State<T>::GetName() {
    return _state_name;
}