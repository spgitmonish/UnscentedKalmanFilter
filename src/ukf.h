#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  // Initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // If this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // If this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  // State covariance matrix
  MatrixXd P_;

  // Predicted sigma points matrix
  MatrixXd Xsig_pred_;

  // Time when the state is true, in us(microseconds)
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  // The current NIS for radar
  double NIS_radar_;

  // The current NIS for laser
  double NIS_laser_;

  // Constructor
  UKF();

  // Destructor
  virtual ~UKF();

  // Function which processes each measurement
  void ProcessMeasurement(MeasurementPackage meas_package);

  // Prediction Predicts sigma points, the state, and the state covariance
  void Prediction(double delta_t);

  // Updates the state and the state covariance matrix using a laser measurement
  void UpdateLidar(MeasurementPackage meas_package);

  // Updates the state and the state covariance matrix using a radar measurement
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* UKF_H */
