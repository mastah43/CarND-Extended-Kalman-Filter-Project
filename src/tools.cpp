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
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        cerr << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){
        VectorXd estimation = estimations[i];
        VectorXd groundTruthCur = ground_truth[i];
        for (int j=0; j<4; j++) {
            double deviation = estimation(j) - groundTruthCur(j);
            rmse(j) += deviation*deviation;
        }
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);

    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    //check division by zero
    if ((px == 0) && (py == 0)) {
        cerr << "position 0,0 faced";
        return Hj;
    }

    double square_sum = px*px + py*py;
    double dist = sqrt(square_sum);
    Hj(0,0) = px/dist;
    Hj(0,1) = py/dist;
    Hj(1,0) = -py/square_sum;
    Hj(1,1) = px/square_sum;
    Hj(2,0) = py*(vx*py - vy*px)/pow(square_sum, 1.5);
    Hj(2,1) = px*(vy*px - vx*py)/pow(square_sum, 1.5);
    Hj(2,2) = px/dist;
    Hj(2,3) = py/dist;
    return Hj;
}
