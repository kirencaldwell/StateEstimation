#ifndef UTILITIES_H_ 
#define UTILITIES_H_

#include <random>
#include "Eigen/Dense"
#include "unsupported/Eigen/MatrixFunctions"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;


struct normal_random_variable
{
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
    }
};
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
VectorXd GetMvnRnd(MatrixXd R);
#endif