#ifndef UTILITIES_H_ 
#define UTILITIES_H_

#include "Eigen/Dense"
#include "unsupported/Eigen/MatrixFunctions"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;


MatrixXd skew(VectorXd t); 
VectorXd vee(MatrixXd a);
MatrixXd RotateVector(VectorXd x);
VectorXd RotationToVector(MatrixXd R);
MatrixXd AssignSubspace(MatrixXd A, int row, int col, MatrixXd a);
MatrixXd GetSubspace(MatrixXd A, int rowi, int rowf, int coli, int colf);
VectorXd GetSubspace(VectorXd a, int s, int f);
std::string CreateLoggingString(double x);
std::string CreateLoggingString(VectorXd x);
std::string CreateLoggingString(MatrixXd x);
void PrintTimeUpdate(double t, MatrixXd R, int K);
#endif