#include <iostream>
#include <sstream>
#include <iomanip>

#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(std::string name) {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // set state dimension
  n_x_ = 5;

  // set augmented dimension
  n_aug_ = 7;

  // define spreading parameter
  lambda_ = 3.0 - n_aug_;

  // set measurement dimension:
  // lidar can measure px and py,
  // radar can measure r, phi, and r_dot
  n_z_lidar_ = 2;
  n_z_radar_ = 3;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // Initial covariance matrix:
  // As we only initialize the position, we set
  // the variances for the speed, yaw angle, and yaw rate
  // to relatively high values:
  // - Speed: 5 m/s (18 kph) standard deviation
  // - Yaw angle: 180 deg (pi radians) standard deviation
  // - Yaw rate: 12 deg/s (0.2 rad/s) standard deviation
  P_ = MatrixXd(n_x_, n_x_);
  P_ << 4, 0, 0,  0,  0,
        0, 4, 0,  0,  0,
        0, 0, 25, 0,  0,
        0, 0, 0,  10, 0,
        0, 0, 0,  0,  0.04;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  is_initialized_ = false;

  time_us_ = 0;

  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2*n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  Xsig_aug_ = Eigen::MatrixXd(n_aug_, 2*n_aug_ + 1);
  Xsig_aug_.fill(0.0);

  // Allocate and compute the weights
  weights_ = VectorXd(2*n_aug_+1);
  computeWeights();

  // Initialize the measurement noise matrices.
  R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  R_lidar_ << (std_laspx_*std_laspx_), 0,
              0, (std_laspy_*std_laspy_);
  R_radar_ << (std_radr_*std_radr_), 0, 0,
              0, (std_radphi_*std_radphi_), 0,
              0, 0, (std_radrd_*std_radrd_);

  // Initialize the lidar measurement matrix
  H_lidar_ = MatrixXd(n_z_lidar_, n_x_);
  H_lidar_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  //-------------- Debug stuff ------------
  name_ = name;

  // See http://www.cplusplus.com/reference/iomanip/setfill/
  std::stringstream ss;
  // ss << "/tmp/ukf_debug_" << std::setfill('0') << std::setw(2) << id << ".txt";
  ss << "/tmp/ukf_debug_" << name_ << ".txt";
  out_file_ = std::make_shared<std::ofstream>(ss.str(), std::ios::out);
  *out_file_ << "*** UKF for " << name_ << " ***\n";

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  switch(meas_package.sensor_type_) {
    case MeasurementPackage::LASER:
      *out_file_ << "\n* LASER at t = " << meas_package.timestamp_ << "\n";
      *out_file_ << "* Measured Cartesian position: (" << meas_package.raw_measurements_(0) << ", " <<
        meas_package.raw_measurements_(1) << ")\n";
      break;
    case MeasurementPackage::RADAR:
      *out_file_ << "\n* RADAR at t = " << meas_package.timestamp_ << "\n";
      double range = meas_package.raw_measurements_(0);
      double azimuth = meas_package.raw_measurements_(1);
      *out_file_ << "* Measured Cartesian position: (" << range*std::cos(azimuth) << ", " <<
        range*std::sin(azimuth) << ")\n";
      break;
  }

  // Check if the UKF has already been initialized.
  // If so, predict to the current measurement time and
  // then update by means of the measurement.
  // If not, initialize the UKF.
  if (!is_initialized_) {
    *out_file_ << "* Initializing the UKF with measurement at t =  " << meas_package.timestamp_ << "\n";
    // Set is_initialized_ to true after initializing.

    // Initialize with the respective formulas depending on the sensor type.
    // Only initialize the position.
    // Even though Radar gives the velocity,
    // it is the radial relative velocity, but not the speed which is contained in the state.
    // It could even be the case that a car is moving tangentially with a high speed,
    // and Radar measures a range rate of (approximately) 0.
    switch (meas_package.sensor_type_) {
      case MeasurementPackage::LASER:
        x_(0) = meas_package.raw_measurements_(0);  // x-position
        x_(1) = meas_package.raw_measurements_(1);  // y-position
        break;
      case MeasurementPackage::RADAR:
        double range = meas_package.raw_measurements_(0);
        double azimuth = meas_package.raw_measurements_(1);
        x_(0) = range*std::cos(azimuth);    // x-position
        x_(1) = range*std::sin(azimuth);    // y-position
        break;
    }

    *out_file_ << "* Position initialized to (" << getXPosition() << ", " << getYPosition() << ")\n";
    is_initialized_ = true;
    return;
  } else {
    double delta_t = meas_package.timestamp_ - time_us_;
    delta_t *= 1.0e-6;  // Convert from us to seconds
    //
    // Predict even when delta_t == 0.
    // Reason: This ensures that the 'predicted' sigma points are distributed
    // correctly about the most recent mean state x and covariance P.
    // In other words: The correction (measurement update) step only updates
    // x and P, but not the corresponding (predicted) sigma points, which are
    // however needed for the computation of the cross-correlation matrix T.
    // That means that when two measurements arrive at the same time, we have
    // to either
    //   - predict for 0 seconds (as done here), or
    //   - recompute the (non-augmented) state sigma points (Xsig_pred_)
    //     based on the updated (corrected) x and P
    //     (not done here for the sake of a clear control flow)
    //
    // The prediction for 0 seconds is numerically perfectly fine.
    // It does not even introduce additional (process) noise/uncertainty,
    // but is rather just used to get the correct sigma points without introducing
    // an if-else-branch
    //
    Prediction(delta_t);
    time_us_ = meas_package.timestamp_;
    switch(meas_package.sensor_type_) {
        case MeasurementPackage::LASER:
          *out_file_ << "* Updating LASER\n";
          UpdateLidar(meas_package);
          break;
        case MeasurementPackage::RADAR:
          *out_file_ << "* Updating RADAR\n";
          UpdateRadar(meas_package);
          break;
      }
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  *out_file_ << "* Predicting for dt = " << delta_t << " seconds (" << delta_t*1.0e6 << " us)\n";

  if (!is_initialized_) {
    *out_file_ << "Error: Cannot predict because the UKF is not yet initialized.\n";
    std::cerr << "Error: UKF is not yet initialized!\n";
    return;
  }

  // First of all, initialize the (augmented) sigma points (calligraphic X)
  // based on the current mean state x and its covariance P.
  initializeAugmentedStateSigmaPoints();

  // Predict the (augmented) sigma points by means of the plant model
  // (or state transition model) in order to get the predicted sigma points.
  predictStateSigmaPoints(delta_t);

  // Based on the predicted sigma points, compute the new mean
  // state x and its covariance P.
  computeMeanStateAndCovarianceFromPredictedStateSigmaPoints();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Check input sensor modality.
  if (meas_package.sensor_type_ != MeasurementPackage::LASER) {
    std::cerr << "Error in UpdateLidar(...): Invalid sensor type\n";
    return;
  }

  //
  // See Lesson 03 (EKF), Concept 13
  //

  // Predicted (aka expected) measurement.
  VectorXd z_pred = H_lidar_ * x_;

  // Actual measurement.
  VectorXd z = VectorXd(n_z_lidar_);
  z(0) = meas_package.raw_measurements_(0);
  z(1) = meas_package.raw_measurements_(1);

  // Innovation (no need to normalize any angle, as we only have position data here).
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_lidar_.transpose();
  MatrixXd S = H_lidar_ * P_ * Ht + R_lidar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  x_(3) = normalizeAngle(x_(3));
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H_lidar_) * P_;

  // Compute and export the NIS (normalized innovation squared) value.
  double nis = y.transpose() * Si * y;
  *out_file_ << "* LIDAR-NIS: " << nis << "\n";
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // Check input sensor modality.
  if (meas_package.sensor_type_ != MeasurementPackage::RADAR) {
    std::cerr << "Error in UpdateRadar(...): Invalid sensor type\n";
    return;
  }

  //
  // See Lesson 04, Concept 26.
  //

  // Create a matrix of sigma points in the measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2*n_aug_ + 1);
  Zsig.fill(0.0);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.fill(0.0);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
  S.fill(0.0);

  // Check dimensions.
  if (Xsig_pred_.cols() != 2*n_aug_ + 1) {
    std::cerr << "Xsig_pred_ has invalid number of columns\n";
    return;
  }

  // Transform the sigma points into the Radar measurement space.
  for (int ii=0; ii<Xsig_pred_.cols(); ii++) {
    double px = Xsig_pred_(0, ii);
    double py = Xsig_pred_(1, ii);
    double v = Xsig_pred_(2, ii);
    double psi = Xsig_pred_(3, ii);
    double psi_dot = Xsig_pred_(4, ii);

    // Apply the radar measurement equation
    double rho = std::sqrt(px*px + py*py);
    double phi = std::atan2(py, px);
    double rho_dot = v * (px*std::cos(psi) + py*std::sin(psi)) / rho;

    Zsig(0, ii) = rho;
    Zsig(1, ii) = phi;
    Zsig(2, ii) = rho_dot;
  }

  // Calculate the mean predicted (or actually expected) measurement.
  for (int ii=0; ii<Zsig.cols(); ii++) {
    z_pred += Zsig.col(ii) * weights_(ii);
  }

  // Calculate the innovation covariance matrix S.
  //    Part 1: Uncertainty in the state space, 'projected' (by means of sigma
  //    points) into the measurement space.
  for (int ii=0; ii<Zsig.cols(); ii++) {
    MatrixXd z_deviation = Zsig.col(ii) - z_pred;
    // Normalize the angle
    z_deviation(1) = normalizeAngle(z_deviation(1));
    MatrixXd z_deviation_transpose = z_deviation.transpose();
    MatrixXd tmp = z_deviation * z_deviation_transpose;
    S += tmp * weights_(ii);
  }
  //    Part 2: Additional uncertainty from the actual measurement process.
  S += R_radar_;

  //
  // See Lesson 04, Concept 29.
  //

  // Calculate the cross correlation matrix.
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
  Tc.fill(0.0);
  for (int ii=0; ii<2*n_aug_ + 1; ii++) {
    MatrixXd x_deviation = Xsig_pred_.col(ii) - x_;
    // Normalize angle
    x_deviation(3) = normalizeAngle(x_deviation(3));
    MatrixXd z_deviation = Zsig.col(ii) - z_pred;
    // Normalize angle
    z_deviation(1) = normalizeAngle(z_deviation(1));
    MatrixXd z_deviation_transpose = z_deviation.transpose();
    Tc += (x_deviation * z_deviation_transpose) * weights_(ii);
  }

  // Calculate the Kalman gain K.
  MatrixXd K = Tc * S.inverse();
  MatrixXd Kt = K.transpose();

  // Create a vector to hold the actual measurement.
  VectorXd z = VectorXd(n_z_radar_);
  double range = meas_package.raw_measurements_(0);
  double azimuth = meas_package.raw_measurements_(1);
  double range_rate = meas_package.raw_measurements_(2);
  z(0) = range;
  z(1) = azimuth;
  z(2) = range_rate;

  // Update the state mean x and the covariance matrix P.
  VectorXd z_diff = z - z_pred;
  z_diff(1) = normalizeAngle(z_diff(1));
  x_ += K*z_diff;
  x_(3) = normalizeAngle(x_(3));
  P_ -= K*S*Kt;

  // Compute and export the NIS (normalized innovation squared) value.
  double nis = z_diff.transpose() * S.inverse() * z_diff;
  *out_file_ << "* RADAR-NIS: " << nis << "\n";

}

void UKF::computeWeights() {
  //
  // See Lesson 04, Concept 23.
  //

  // Check dimensions
  if (weights_.size() != 2*n_aug_ + 1) {
    std::cerr << "Error: weights_ has invalid dimensions, will not compute the weights.\n";
    std::cerr << "Actual weight size number is " << weights_.size() << ", expected " << (2*n_aug_+1) << "\n";
    return;
  }

  double weight_0 = lambda_ / (lambda_ + n_aug_);
  double other_weights = 0.5 / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int ii=1; ii<2*n_aug_+1; ii++) {
    weights_(ii) = other_weights;
  }
}

void UKF::initializeAugmentedStateSigmaPoints() {
  //
  // See Lesson 04, Concept 17.
  //

  // Create process noise matrix Q.
  MatrixXd Q = MatrixXd(2, 2);
  Q << (std_a_*std_a_), 0,
       0, (std_yawdd_, std_yawdd_);

  // Create augmented covariance matrix P_aug.
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q;

  // Create lower triangluar
  // square root matrix A_aug (such that P_aug = A_aug * A_aug^T)
  // by means of Cholesky decomposition.
  MatrixXd A_aug = P_aug.llt().matrixL();
  A_aug = A_aug * std::sqrt(lambda_ + n_aug_);

  // Create augmented mean state.
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0.0;   // process noise mean is 0
  x_aug(6) = 0.0;   // process noise mean is 0

  // Create augmented sigma points for the state.
  Xsig_aug_.fill(0.0);
  for (int cc=0; cc<n_aug_; cc++) {
      Xsig_aug_.col(cc+1) = A_aug.col(cc);
      Xsig_aug_.col(cc+1+n_aug_) = A_aug.col(cc) * (-1.0);
  }
  for (int cc=0; cc<2*n_aug_ + 1; cc++) {
      Xsig_aug_.col(cc) += x_aug;
  }

}

void UKF::predictStateSigmaPoints(double delta_t) {
  //
  // See Lesson 04, Concept 20.
  //

  double delta_t2 = 0.5*delta_t*delta_t;  // 1/2 * delta_t squared

  for (int cc=0; cc<Xsig_aug_.cols(); cc++) {
    // predict sigma points
    VectorXd x_aug = Xsig_aug_.col(cc);
    double px = x_aug(0);
    double py = x_aug(1);
    double v = x_aug(2);
    double psi = x_aug(3);
    double psi_dot = x_aug(4);
    double nu_a = x_aug(5);
    double nu_psi_dd = x_aug(6);

    VectorXd x_pred = x_aug.head(n_x_);

    // avoid division by zero
    if (std::abs(psi_dot) < 1.0e-6) {
      // Straight driving

      // Deterministic part for px and py
      x_pred(0) += v*std::cos(psi) * delta_t;
      x_pred(1) += v*std::sin(psi) * delta_t;
    } else {
      // Deterministic part for px and py
      x_pred(0) += v/psi_dot*(std::sin(psi + psi_dot*delta_t) - std::sin(psi));
      x_pred(1) += v/psi_dot*(-std::cos(psi + psi_dot*delta_t) + std::cos(psi));
    }
    // Deterministic part for v, psi, and psi_dot
    x_pred(2) += 0;
    x_pred(3) += psi_dot * delta_t;
    x_pred(4) += 0;

    // Add the stochastic part, which is the same for both plant models (psi_dot == 0 and psi_dot != 0)
    x_pred(0) += delta_t2 * std::cos(psi) * nu_a;
    x_pred(1) += delta_t2 * std::sin(psi) * nu_a;
    x_pred(2) += delta_t * nu_a;
    x_pred(3) += delta_t2 * nu_psi_dd;
    x_pred(4) += delta_t * nu_psi_dd;

    // write predicted sigma points into respective column
    Xsig_pred_.col(cc) = x_pred;
  }

}

void UKF::computeMeanStateAndCovarianceFromPredictedStateSigmaPoints() {
  //
  // See Lesson 04, Concept 23.
  //

  // Predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    x_diff(3) = normalizeAngle(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

double UKF::normalizeAngle(double angle) const {
  constexpr double two_pi = 2.0 * M_PI;
  while (angle > M_PI) {
    angle -= two_pi;
  }
  while (angle < -M_PI) {
    angle += two_pi;
  }

  return angle;
}
