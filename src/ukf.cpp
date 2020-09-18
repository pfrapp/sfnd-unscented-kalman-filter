#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.fill(0.0);

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

  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2*n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  Xsig_aug_ = Eigen::MatrixXd(n_aug_, 2*n_aug_ + 1);
  Xsig_aug_.fill(0.0);

  // Compute the weights
  computeWeights();

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}

void UKF::computeWeights() {
  //
  // See Lesson 04, Concept 23.
  //

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

