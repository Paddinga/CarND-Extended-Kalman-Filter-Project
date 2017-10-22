#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;
// check the validity of the following inputs:
//  * the estimation vector size should not be zero
//  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }
//accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
//coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }
//calculate the mean
    rmse = rmse/estimations.size();
//calculate the squared root
    rmse = rmse.array().sqrt();
//return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);
//recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    if (fabs(px) <= 0.0001 && fabs(py) <= 0.0001){
        px = 0.0001;
        py = 0.0001;
   }
//prepare values
    float div1 = px * px + py * py;
    float div2 = sqrt(div1);
    float xyz = sqrt(div1);
    float div3 = div1 * div2;
//check division by zero
    if(fabs(div1) < 0.0001){
        div1 = 0.0001;
    }
//compute the Jacobian matrix
    Hj <<   px/div2, py/div2, 0, 0,
            -py/div1, px/div1, 0, 0,
            py*(vx*py-vy*px)/div3, px*(vy*px-vx*py)/div3, px/div2, py/div2;
    return Hj;
}
