#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    
  /**
  TODO:
    * Calculate the RMSE here.
  */
    // initialise a rmse vector to return the values
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    
    // Check to ensure estimation vector size is not zero
    if(estimations.size() != 0)
    {
        // check to ensure both the vectors are of same size
        if(estimations.size() == ground_truth.size())
        {
            // sum of squared residuals for the vectors
            for(int i = 0; i < estimations.size(); ++i)
            {
                VectorXd residual = estimations[i] - ground_truth[i];
                residual = residual.array() * residual.array();
                rmse += residual;
            }
            
            // calculate the mean
            rmse = rmse / estimations.size();
            
            //calculate mean squared root
            rmse = rmse.array().sqrt();
        }
    }
  
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    // Initialise Jacobian Matrix
    MatrixXd Hj(3,4);
    
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    float c1 = (px * px) + (py * py);
    float c2 = sqrt(c1);
    float c3 = c1 * c2;
    
    // check for division by zero
    if(fabs(c1) < 0.0001)
    {
        //std::cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        Hj << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;
        return Hj;
    }
    
    
    
    // calculate jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
          -(py / c1), (px / c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    
    return Hj;
    
}
