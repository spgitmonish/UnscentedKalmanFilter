#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  // Vector storing the results of the calculation for each pass
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Check the validity of the following inputs:
  //  * The estimation vector size should not be zero
  //  * The estimation vector size should equal ground truth vector size
  if((estimations.size() == 0) || (estimations.size() != ground_truth.size()))
  {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // Accumulate the root mean squared errors
  for(int i = 0; i < estimations.size(); ++i)
  {
    // Get the residual
    VectorXd residual = estimations[i] - ground_truth[i];

    // Calculate the square
    residual = residual.array() * residual.array();

    // Calculate the sum
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse/estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();

  // Return the result
  return rmse;
}
