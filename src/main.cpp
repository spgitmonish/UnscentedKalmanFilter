
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// NOTE : Flip this to 1 to use it with the simulator.
//        The simulator needs to be of size 1600x900 with Simple mode
#define DEBUG_SIMULATOR_OUTPUT 0

// Function which checks if the user has provided the correct arguments
void check_arguments(int argc, char* argv[])
{
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // Make sure the user has provided input and output files
  if (argc == 1)
  {
    cerr << usage_instructions << endl;
  }
  else if (argc == 2)
  {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  }
  else if (argc == 3)
  {
    has_valid_args = true;
  }
  else if (argc > 3)
  {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args)
  {
    exit(EXIT_FAILURE);
  }
}

// Function to check if the files specified can be opened
void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name)
{
  if (!in_file.is_open())
  {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open())
  {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

// Main function
int main(int argc, char* argv[])
{
  // Check the arguments
  check_arguments(argc, argv);

  // Input and output files
  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  // Vectors of measurements and the ground truth
  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // Prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line))
  {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // Reads first element from the current line
    iss >> sensor_type;

    // Lidar measurement
    if (sensor_type.compare("L") == 0)
    {
      // Read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);

      // Lidar measures px, py
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;

      // Get the timestamp
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    // Radar measurement
    else if (sensor_type.compare("R") == 0)
    {
      // Read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);

      // Radar measures ro, phi and ro_dot
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;

      // Get the timestamp
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
      // Read ground truth data to compare later
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      float yaw_gt;
      float yawrate_gt;

      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      iss >> yaw_gt;
      iss >> yawrate_gt;

      gt_package.gt_values_ = VectorXd(6);
      gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yawrate_gt;
      gt_pack_list.push_back(gt_package);
  }

// Do a binary search against different values of longitudinal and yaw acceleration
// noise standard deviation
#if DO_A_YAWDD_SD_BIN_SEARCH
  // String for file name modification(empty to start with)
  string new_out_file_name = "";

  for(double std_a_test = 0.05; std_a_test < 3.0; )
  {
    for(double std_yawdd_test = 0.05; std_yawdd_test < 2.0; )
    {
      // Convert the values into string
      string std_a_str = to_string(std_a_test);
      string std_yawdd_str = to_string(std_yawdd_test);

      // Create the new file name based on the user input
      new_out_file_name = out_file_name_;

      // Get position using length of the file name to insert
      // the new string at the end
      size_t pos_to_insert = new_out_file_name.length() - 4;

      new_out_file_name.insert(pos_to_insert, (std_a_str + std_yawdd_str));
      cout << new_out_file_name << "\n" << endl;

      // Using ofstream class create a new file
      // to write data to file open for output operations
      ofstream out_file_(new_out_file_name.c_str(), ofstream::out);

      // Create a UKF instance
      UKF ukf;

      // Modify the values of std_a_ and std_yawdd_
      ukf.std_a_ = std_a_test;
      ukf.std_yawdd_ = std_yawdd_test;

      // Used to compute the RMSE later
      vector<VectorXd> estimations;
      vector<VectorXd> ground_truth;

      // Start filtering from the 2nd frame(the speed is unknown in the first frame)
      size_t number_of_measurements = measurement_pack_list.size();

      // Column names for output file
      out_file_ << "time_stamp" << "\t";
      out_file_ << "px_state" << "\t";
      out_file_ << "py_state" << "\t";
      out_file_ << "v_state" << "\t";
      out_file_ << "yaw_angle_state" << "\t";
      out_file_ << "yaw_rate_state" << "\t";

      out_file_ << "sensor_type" << "\t";
      out_file_ << "NIS" << "\t";

      out_file_ << "px_measured" << "\t";
      out_file_ << "py_measured" << "\t";

      out_file_ << "px_ground_truth" << "\t";
      out_file_ << "py_ground_truth" << "\t";
      out_file_ << "vx_ground_truth" << "\t";
      out_file_ << "vy_ground_truth" << "\t";
      out_file_ << "yaw_ground_truth" << "\t";
      out_file_ << "yawrate_ground_truth" << "\t";

      out_file_ << "px_estimate" << "\t";
      out_file_ << "py_estimate" << "\t";
      out_file_ << "vx_estimate" << "\t";
      out_file_ << "vy_estimate" << "\t";
      out_file_ << "yaw_estimate" << "\t";
      out_file_ << "yawrate_estimate" << "\n";

      // Call the UKF-based fusion for each measurement
      for (size_t k = 0; k < number_of_measurements; ++k)
      {
        ukf.ProcessMeasurement(measurement_pack_list[k]);

        // Timestamp
        out_file_ << measurement_pack_list[k].timestamp_ << "\t";

        // Output the state vector
        out_file_ << ukf.x_(0) << "\t"; // pos1 - est
        out_file_ << ukf.x_(1) << "\t"; // pos2 - est
        out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
        out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
        out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

        // Output lidar and radar specific data
        if ((measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) && ukf.use_laser_)
        {
          // Sensor type Lidar
          out_file_ << "lidar" << "\t";

          // NIS value
          out_file_ << ukf.NIS_lidar_ << "\t";

          // Output the lidar sensor measurement px and py
          out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
          out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";

        }
        else if ((measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) && ukf.use_radar_)
        {
          // Sensor type Radar
          out_file_ << "radar" << "\t";

          // NIS value
          out_file_ << ukf.NIS_radar_ << "\t";

          // Output radar measurement in cartesian coordinates
          float ro = measurement_pack_list[k].raw_measurements_(0);
          float phi = measurement_pack_list[k].raw_measurements_(1);
          out_file_ << ro * cos(phi) << "\t"; // px measurement
          out_file_ << ro * sin(phi) << "\t"; // py measurement
        }

        // Output the ground truth
        out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(3) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(4) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(5) << "\t";

        // Convert ukf x vector to cartesian to compare to ground truth
        VectorXd ukf_x_cartesian_ = VectorXd(6);

        float x_estimate_ = ukf.x_(0);
        float y_estimate_ = ukf.x_(1);
        float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
        float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
        float yaw_estimate_ = ukf.x_(3);
        float yawrate_estimate_ = ukf.x_(4);

        // Output the estimations
        out_file_ << x_estimate_ << "\t";
        out_file_ << y_estimate_ << "\t";
        out_file_ << vx_estimate_ << "\t";
        out_file_ << vy_estimate_ << "\t";
        out_file_ << yaw_estimate_ << "\t";
        out_file_ << yawrate_estimate_ << "\n";

        ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_, yaw_estimate_, yawrate_estimate_;

        // Store the values in the respective object vectors
        estimations.push_back(ukf_x_cartesian_);
        ground_truth.push_back(gt_pack_list[k].gt_values_);
      }

      // Compute the accuracy (RMSE)
      Tools tools;
      cout << "RMSE" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

      // Close files
      if (out_file_.is_open()) {
        out_file_.close();
      }

      // Incremented the process noise yaw acceleration SD
      std_yawdd_test += 0.1;
    }
    // Incremented the process noise longitudinal acceleration SD
    std_a_test += 0.1;
  }
#else
  // Create a UKF instance
  UKF ukf;

  // Used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // Start filtering from the 2nd frame(the speed is unknown in the first frame)
  size_t number_of_measurements = measurement_pack_list.size();

  // Column names for output file
  out_file_ << "time_stamp" << "\t";
  out_file_ << "px_state" << "\t";
  out_file_ << "py_state" << "\t";
  out_file_ << "v_state" << "\t";
  out_file_ << "yaw_angle_state" << "\t";
  out_file_ << "yaw_rate_state" << "\t";

  out_file_ << "sensor_type" << "\t";
  out_file_ << "NIS" << "\t";

  out_file_ << "px_measured" << "\t";
  out_file_ << "py_measured" << "\t";

  out_file_ << "px_ground_truth" << "\t";
  out_file_ << "py_ground_truth" << "\t";
  out_file_ << "vx_ground_truth" << "\t";
  out_file_ << "vy_ground_truth" << "\t";
  out_file_ << "yaw_ground_truth" << "\t";
  out_file_ << "yawrate_ground_truth" << "\t";

  out_file_ << "px_estimate" << "\t";
  out_file_ << "py_estimate" << "\t";
  out_file_ << "vx_estimate" << "\t";
  out_file_ << "vy_estimate" << "\t";
  out_file_ << "yaw_estimate" << "\t";
  out_file_ << "yawrate_estimate" << "\n";

  // Call the UKF-based fusion for each measurement
  for (size_t k = 0; k < number_of_measurements; ++k)
  {
    ukf.ProcessMeasurement(measurement_pack_list[k]);

    // Timestamp
    out_file_ << measurement_pack_list[k].timestamp_ << "\t";

    // Output the state vector
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // Output lidar and radar specific data
    if ((measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) && ukf.use_laser_)
    {
      // Sensor type Lidar
      out_file_ << "lidar" << "\t";

      // NIS value
      out_file_ << ukf.NIS_lidar_ << "\t";

      // Output the lidar sensor measurement px and py
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";

    }
    else if ((measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) && ukf.use_radar_)
    {
      // Sensor type Radar
      out_file_ << "radar" << "\t";

      // NIS value
      out_file_ << ukf.NIS_radar_ << "\t";

      // Output radar measurement in cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // px measurement
      out_file_ << ro * sin(phi) << "\t"; // py measurement
    }

    // Output the ground truth
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(4) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(5) << "\t";

    // Convert ukf x vector to cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(6);

    float x_estimate_ = ukf.x_(0);
    float y_estimate_ = ukf.x_(1);
    float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
    float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
    float yaw_estimate_ = ukf.x_(3);
    float yawrate_estimate_ = ukf.x_(4);

    // Output the estimations
    out_file_ << x_estimate_ << "\t";
    out_file_ << y_estimate_ << "\t";
    out_file_ << vx_estimate_ << "\t";
    out_file_ << vy_estimate_ << "\t";
    out_file_ << yaw_estimate_ << "\t";
    out_file_ << yawrate_estimate_ << "\n";

    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_, yaw_estimate_, yawrate_estimate_;

    // Store the values in the respective object vectors
    estimations.push_back(ukf_x_cartesian_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // Compute the accuracy (RMSE)
  Tools tools;
  VectorXd rmse_output = tools.CalculateRMSE(estimations, ground_truth);

#if DEBUG_SIMULATOR_OUTPUT
  cout << "RMSE" << endl;
  for(size_t rmse_index = 0; rmse_index < rmse_output.size(); rmse_index++)
  {
    cout << rmse_output(rmse_index) << endl;
  }
#else
  cout << "Accuracy - RMSE:" << endl << rmse_output << endl;
#endif

  // Close files
  if (out_file_.is_open()) {
    out_file_.close();
  }
#endif // #if DO_LONG_A_YAWDD_BIN_SEARCH

  if (in_file_.is_open()) {
    in_file_.close();
  }

  cout << "Done!" << endl;
  return 0;
}
