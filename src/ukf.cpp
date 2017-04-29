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

  // Initalized to false
  is_initialized_ = false;

  // If this is false, laser measurements will be ignored
  use_laser_ = true;

  // If this is false, radar measurements will be ignored
  use_radar_ = true;

  // Initialize state dimension(px, py, v, yaw, yaw_rate)
  n_x_ = 5;

  // Initial state vector
  x_ = VectorXd(n_x_);

  // Initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Initialize the state covariance matrix 'P'
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Initialize augmented state dimension(px, py, v, yaw, yaw_rate, a, yawdd)
  // NOTE: Process noise has a non-linear effect, so it is represented by
  //       sigma points as well
  n_aug_ = 7;

  // Initalize spreading parameter lamda
  lambda_ = 3 - n_aug_;

  // Initalize the weights vector
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Initialize augmented state(mean) vector
  x_aug_ = VectorXd(n_aug_);

  // Initialize augmented state covariance
  P_aug_ = MatrixXd(n_aug_, n_aug_);

  // Initialize Augmented Sigma Point Matrix(n_aug_, 2*n_aug_ + 1 dimensions)
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Initialize Predicted Sigma Point Matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

#if !DO_A_YAWDD_SD_BIN_SEARCH
  // NOTE: These values were set after doing a binary search and looking at the
  //       RMSE variations for different combinations and picking the optimal
  //       combination which gives the lowest RMSE for px, py, vx, vy
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.40;

  // Process noise standard deviation yaw acceleration in rad/s^2(yaw double dot)
  std_yawdd_ = 1.20;
#endif

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

  // Process noise covariance matrix
  // Longitudnal accelaration and yaw accelaration noise variance
  Q_ = MatrixXd(2, 2);
  Q_ << std_a_*std_a_, 0,
        0, std_yawdd_*std_yawdd_;

  // Lidar Measurement noise covariance matrix
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  // Measurement matrix for Laser
  H_lidar_ = MatrixXd(2, 5);
  H_lidar_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  // Radar Measurement noise covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_, 0, 0,
              0, std_radphi_, 0,
              0, 0, std_radrd_;

  // Radar measures rho, phi, rho_dot
  n_z_radar_ = 3;

  // Predicted radar measurement sigma points
  Zsig_pred_radar_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

  // Predicted radar measurement vector
  z_pred_radar_ = VectorXd(n_z_radar_);

  // Predicted radar measurement covariance matrix
  S_pred_radar_ = MatrixXd(n_z_radar_, n_z_radar_);

  // Set the NIS_lidar_ and NIS_radar_ values to 0.0
  NIS_lidar_ = 0.0;
  NIS_radar_ = 0.0;
}

UKF::~UKF() {}

// Process each measurement using UKF
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Check if the initialization is done i.e. this is the first measruement
  if (!is_initialized_)
  {
    // Initialize the state ekf_.x_ with the first measurement.
    // NOTE: For an unscented kalman filter, the idea is to predict the state
    //       vector with px, py, v, yaw, yaw_rate
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

    // Perform a one time calculation of the weights
    CalculateWeights();

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
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_)
  {
    // Lidar updates
    UpdateLidar(meas_package);
  }
  else if((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_)
  {
    // Radar updates
    UpdateRadar(meas_package);
  }
}

// Calculate the weights which is used in determining predicted state/covariance
// predicted measurement state/covariance using the respective sigma points
void UKF::CalculateWeights()
{
  // First weight
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights_(0) = weight_0;

  // Remaining weights
  for (int w = 1; w < 2*n_aug_ + 1; w++)
  {
    double weight = 0.5/(lambda_ + n_aug_);
    weights_(w) = weight;
  }
}

// Predicts sigma points, the state, and the state covariance matrix.
// delta_t: The change in time (in seconds) between
//          the last measurement and this one.
void UKF::Prediction(double delta_t)
{
  // Generate augmented sigma points
  GenerateAugmentedSigmaPoints();

  // Generate predicted sigma points
  GeneratePredictedSigmaPoints(delta_t);

  // Generate process state and covariance matrices from predicted sigma points
  GenerateProcessStateAndCovariance();
}

// Function to generate augmented sigma points
void UKF::GenerateAugmentedSigmaPoints()
{
  // Create augmented mean state with the noise
  // NOTE: The noise has zero mean and sigma^2 variance
  x_aug_.fill(0.0);
  x_aug_.head(n_x_) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  // Create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_.bottomRightCorner(2, 2) = Q_;

  // Calculate square root of augmented state covariance matrix 'P'
  L_ = P_aug_.llt().matrixL();

  // Initialize with 0s
  Xsig_aug_.fill(0.0);

  // Set first column of sigma point matrix
  Xsig_aug_.col(0) = x_aug_;

  // Set remaining sigma points
  for (int i = 0; i < n_aug_; i++)
  {
     Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L_.col(i);
     Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L_.col(i);
  }
}

// Function to generate predicted sigma points
void UKF::GeneratePredictedSigmaPoints(double delta_t)
{
  // Predict sigma points of k+1 using augmented sigma points of state at 'k'
  // Each of the sigma points is passed through the function f(x, v)
  // NOTE 1: There are 2*n_aug + 1 sigma points(i.e number of columns) for the 5 dimensions
  // NOTE 2: The input is 7-D of the augmented state,
  //         output is 5-D for the predicted state
  Xsig_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    // Predicted state position values
    double px_p, py_p;

    // Avoid division by zero
    if (fabs(yawd) > 0.001)
    {
        px_p = p_x + v/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw) );
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else
    {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    // Predicted state velocity, yaw and yaw rate values
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // Add noise for all the components of the state
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // Write predicted sigma point into correct column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
}

// Generate the process state vector and covariance matrix
// using the predicted sigma points
void UKF::GenerateProcessStateAndCovariance()
{
  // For the state vector 'x'
  x_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Predict state mean
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  P_.fill(0.0);
  // For the state covariance matrix 'P'
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // The difference between column 'i'(sigma point for each of the dimension)
    // and the state vector
    VectorXd X_min_x =  Xsig_pred_.col(i) - x_;

    // Normalize the angles
    NormalizeAngles(X_min_x(3));

    // Predict the state covariance
    P_ = P_ + weights_(i) * X_min_x * X_min_x.transpose();
  }
}

// Updates the state and the state covariance matrix using lidar measurements
// NOTE: Lidar data is linear so the steps defined for radar is not
//       required i.e. non-linear to linear conversion
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  // Get the measured vector from the sensor
  VectorXd z_ = meas_package.raw_measurements_;

  // Update the state by using Kalman Filter equations
  VectorXd y = z_ - H_lidar_ * x_;

  // The remaining questions are the same
	MatrixXd Ht = H_lidar_.transpose();
	MatrixXd S = H_lidar_ * P_ * Ht + R_lidar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// New estimate
	x_ = x_ + (K * y);

  // Create the identity matrix of the same size
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // New covariance
	P_ = (I - K * H_lidar_) * P_;

  // Calculate Normalize Innovation Squared for Lidar
  NIS_lidar_ = CalculateNIS(z_, H_lidar_ * x_, S);
}

// Updates the state and the state covariance matrix using radar measurements.
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  // Get the measured vector from the sensor
  VectorXd z_ = meas_package.raw_measurements_;

  // Radar measures rho, phi and rho_dot
  int n_z_ = n_z_radar_;

  // Transform sigma points into measurement space
  for(int i = 0; i < 2*n_aug_ + 1; i++)
  {
    // Get the individual values from the predicted sigma points
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double px2_py2 = sqrt(px*px + py*py);

    // Avoid divide by zero
    if(px2_py2 < 0.0001)
    {
      px2_py2 = 0.0001;
    }

    double rho = px2_py2;
    // NOTE: Using atan2 limits the value between -PI and PI
    double phi = atan2(py, px);
    double rho_dot = ((px*cos(yaw) + py*sin(yaw))*v)/px2_py2;

    // Populate each column of the matrix Z sigma
    Zsig_pred_radar_.col(i) << rho,
                               phi,
                               rho_dot;
  }

  // Calculate mean predicted measurement vector
  z_pred_radar_.fill(0.0);
  for(int i = 0; i < 2*n_aug_ + 1; i++)
  {
    z_pred_radar_ = z_pred_radar_ + Zsig_pred_radar_.col(i) * weights_(i);
  }

  // Calculate measurement covariance matrix S
  S_pred_radar_.fill(0.0);
  for(int i = 0; i < 2*n_aug_ + 1; i++)
  {
    // Difference between Z(measured sigma points) and
    // 'z'(measured mean vector)
    VectorXd Z_diff_z = VectorXd(n_z_);
    Z_diff_z = Zsig_pred_radar_.col(i) - z_pred_radar_;

    // Normalize the angles
    NormalizeAngles(Z_diff_z(1));

    // Calculate the measurement covariance matrix
    S_pred_radar_ = S_pred_radar_ + Z_diff_z * Z_diff_z.transpose() * weights_(i);
  }

  // Add the measurement noise covariance matrix
  S_pred_radar_ = S_pred_radar_ + R_radar_;

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  Tc.fill(0.0);
  // Calculate cross correlation matrix
  for(int i = 0; i < 2*n_aug_ + 1; i++)
  {
    // Calculate vectors with the difference in sigma points and mean state
    // for both the prediction and the predicted measurements
    VectorXd X_diff_x = Xsig_pred_.col(i) - x_;

    // Normalize the angles
    NormalizeAngles(X_diff_x(3));

    VectorXd Z_diff_z = Zsig_pred_radar_.col(i) - z_pred_radar_;

    // Normalize the angles
    NormalizeAngles(Z_diff_z(1));

    Tc = Tc + weights_(i) * X_diff_x * Z_diff_z.transpose();
  }

  // Calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x_, n_z_);
  K = Tc * S_pred_radar_.inverse();

  // Update state mean and covariance matrix
  x_ = x_ + K * (z_ - z_pred_radar_);
  P_ = P_ - K * S_pred_radar_ * K.transpose();

  // Calculate Normalize Innovation Squared for Radar
  NIS_radar_ = CalculateNIS(z_, z_pred_radar_, S_pred_radar_);
}

// Helper function to normalize passed in angle
void UKF::NormalizeAngles(double &angle_to_normalize)
{
  if(angle_to_normalize > M_PI)
  {
    angle_to_normalize = fmod((angle_to_normalize - M_PI), (2*M_PI)) - M_PI;
  }
  else if(angle_to_normalize < -M_PI)
  {
    angle_to_normalize = fmod((angle_to_normalize + M_PI), (2*M_PI)) + M_PI;
  }
}

// Calculate NIS using actual measurement, estimated measurement and
// predicted measurement covariance
double UKF::CalculateNIS(VectorXd z, VectorXd z_pred, MatrixXd S)
{
  VectorXd z_error = z_pred - z;
  double NIS = z_error.transpose() * S.inverse() * z_error;

  return NIS;
}
