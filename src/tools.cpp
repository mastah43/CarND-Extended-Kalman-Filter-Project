#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cerr;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() == 0) {
        cerr << "Estimations are empty" << endl;
        return rmse;
    }
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        cerr << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    double error_px = rmse(0);
    double error_py = rmse(1);
    double error_vx = rmse(2);
    double error_vy = rmse(3);

    if ((error_px > 0.11) || (error_py > 0.11) || (error_vx > 0.52) || (error_vy > 0.52)) {
        cerr << "rmse too high: x=" << error_px << "; y=" << error_py
             << "; vx=" << error_vx << "; vy=" << error_vy << endl;
    }

    return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);

    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    double c1 = px*px + py*py;

    // check division by zero
    if (fabs(c1) < 0.0001) {
        cerr << "distance near 0 faced - using eps";
        c1 = 0.0001;
    }

    double c2 = sqrt(c1);
    double c3 = c1*c2;

    Hj(0,0) = px/c2;
    Hj(0,1) = py/c2;
    Hj(0,2) = 0;
    Hj(0,3) = 0;
    Hj(1,0) = -py/c1;
    Hj(1,1) = px/c1;
    Hj(1,2) = 0;
    Hj(1,3) = 0;
    Hj(2,0) = py*(vx*py - vy*px)/c3;
    Hj(2,1) = px*(vy*px - vx*py)/c3;
    Hj(2,2) = px/c2;
    Hj(2,3) = py/c2;
    return Hj;
}
