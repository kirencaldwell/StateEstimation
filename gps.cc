#include "gps.h"

void Gps::InitGps() {};

VectorXd Gps::RunModel(VectorXd x) {
    return x.head(3);
}