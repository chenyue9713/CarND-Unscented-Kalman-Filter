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
  VectorXd rmse = VectorXd(4);
  rmse << 0,0,0,0;
  
  if(estimations.size() == 0)
  {
    cout << "The estimation vector size should not be zero!" << endl;
    return rmse;
  }
  
  if(estimations.size() != ground_truth.size())
  {
    cout << "The estimation vector size should equal to size of ground Truth vector !" << endl;
    return rmse;
  }
  
  for(unsigned int i = 0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  
  rmse = rmse / estimations.size();
  
  rmse = rmse.array().sqrt();
  if (rmse[0] > 0.09 ||
      rmse[1] > 0.10 ||
      rmse[2] > 0.40 ||
      rmse[3] > 0.30)
  {
  	cout << "rmse exceed the limiting value" << endl;
    cout << "rmse: " << rmse[0] << ", " << rmse[1] << ", "<< rmse[2] << ", "<< rmse[3] << endl;
  }
  
  
  return rmse;
}
