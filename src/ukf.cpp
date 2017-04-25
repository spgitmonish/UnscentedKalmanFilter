#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Initializes Unscented Kalman filter
UKF::UKF() {
  // Variable to keep track of the previous timestamp
  previous_timestamp_ = 0;

  // If this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // If this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Initial state vector
  x_ = VectorXd(5);

  // Initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

// meas_package: The latest measurement data of either radar or lidar.
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  // Check if the initialization is done i.e. this is the first measruement
  if (!is_initialized_)
  {
    // Initialize the state ekf_.x_ with the first measurement.
    // NOTE: For an unscented kalman filter, the idea is to predict the state
    //       vector with px, py, v, yaw, yaw_rate
    x_ = VectorXd(5);

    // Lidar measurement
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // Set the state with the initial location, velocity, yaw and yaw rate
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }
    // Radar measurement
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Angular velocity
      float rho = meas_package.raw_measurements_[0];
      // Angle
      float phi = meas_package.raw_measurements_[1];

      // Convert from polar to cartesian coordinates and initialize state.
      float px = rho * cos(phi);
      float py = rho * sin(phi);

      // Set the state with the initial location, velocity, yaw and yaw rate
      x_ << px, py, 0, 0, 0;
    }

    // Capture the timestamp for the next iteration
		previous_timestamp_ = meas_package.timestamp_;

    // Done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  // Check whether the timestamps match, if they do then we have sensor
  // measurements at the same time. So prediction needs to be done only
  // once.
  if(meas_package.timestamp_ != previous_timestamp_)
  {
    // Compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;

    // Capture the timestamp for the next iteration
    previous_timestamp_ = meas_package.timestamp_;

    // Prediction step
    Prediction(dt);
  }

  // Update step
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    // Lidar updates
    UpdateLidar(meas_package);
  }
  else
  {
    // Radar updates
    UpdateRadar(meas_package);
  }
}

// Predicts sigma points, the state, and the state covariance matrix.
// delta_t: The change in time (in seconds) between
//          the last measurement and this one.
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

// Updates the state and the state covariance matrix using a lidar measurement.
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

// Updates the state and the state covariance matrix using a radar measurement.
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
