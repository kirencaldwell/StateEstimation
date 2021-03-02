#ifndef SYSTEM_MODEL_H_
#define SYSTEM_MODEL_H_

#include <random>
#include <iostream>
#include "../../Eigen/Dense"
#include "../Utilities/utilities.h"
#include "../Utilities/state.h"
#include "../../unsupported/Eigen/MatrixFunctions"
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;

// struct Measurement {
//     std::string sensor_name;
//     VectorXd value;
// };

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
class SystemModel {
    public:
        void AddMeasurement(VectorXd y); 
        VectorXd GetMeasurement();
        void ClearMeasurement();
        void SetModelName(std::string model_name);
        void SetVariance(MatrixXd R);
        virtual MatrixXd GetJacobian(VectorXd x, double h);
        VectorXd GetOutput(VectorXd x);
        std::string GetName();
        
        virtual VectorXd GetNoisyOutput(VectorXd x);
        virtual VectorXd RunModel(VectorXd x);
        // virtual VectorXd RunModel(std::vector<State<VectorXd>> x);
        virtual MatrixXd GetVariance();
        virtual MatrixXd GetVariance(VectorXd x);
    protected:
        std::string _name;
        MatrixXd _R;
        VectorXd _y;
};

#endif