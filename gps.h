#ifndef GPS_H_ 
#define GPS_H_ 

#include "system_model.h"
#include "utilities.h"
#include "unsupported/Eigen/MatrixFunctions"

using namespace Eigen;

class Gps: public SystemModel{
    public:
        void InitGps();
        VectorXd RunModel(VectorXd x) override;
    private:
};
#endif