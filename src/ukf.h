#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  //! Get the x-position estimate (in m)
  double getXPosition() const { return x_(0); }
  
  //! Get the y-position estimate (in m)
  double getYPosition() const { return x_(1); }

  //! Get the speed (velocity magnitude) estimate (in m/s)
  double getSpeed() const { return x_(2); }

  //! Get the yaw angle estimate (in rad)
  double getYaw() const { return x_(3); }

  //! Get the yaw rate estimate (in rad/s)
  double getYawRate() const { return x_(4); }

  //! Get a constant reference to the estimated state.
  const Eigen::VectorXd& getState() const { return x_; };

private:

  /**
   * \brief Compute the weights.
   * \details This has been dealt with in Lesson 04, Concept 23.
   */
  void computeWeights();

  /**
   * \brief Compute the augmented sigma points X based on the mean state x and the state error covariance P.
   * \details The resulting sigma points are stored in Xsig_aug_.
   *          This has been dealt with in Lesson 04, Concept 17.
   */
  void initializeAugmentedStateSigmaPoints();

  /**
   * \brief Predict the sigma points using the plant model (state transition model).
   * \details This has been dealt with in Lesson 04, Concept 20.
   */
  void predictStateSigmaPoints(double delta_t);

  //! Utility function for normalizing an angle so that it is within [-pi, +pi]
  double normalizeAngle(double angle) const;

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // Augmented sigma points matrix
  Eigen::MatrixXd Xsig_aug_;

  // time when the state is true, in us
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
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  // Measurement dimension for Lidar
  int n_z_lidar_;

  // Measurement dimension for Radar
  int n_z_radar_;
};

#endif  // UKF_H