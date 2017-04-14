#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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

  // initial covariance matrix
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
  is_initialized_ = false;
  
  ///* State dimension
  int n_x_ = 5;

  ///* Augmented state dimension
  int n_aug_ = 7;

  ///* Sigma point spreading parameter
  double lambda_ = 3-n_x_;
  
  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  ///* state covariance matrix
  MatrixXd P_ = MatrixXd(n_x_, n_x_);
  P_ << 1000, 0, 0, 0, 0,
          0, 1000, 0, 0, 0,
          0, 0, 1000, 0, 0,
          0, 0, 0, 1000, 0,
          0, 0, 0, 0, 1000;
  
  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {
      weights_(i) = 0.5/(lambda_+n_aug_);
  }
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    previous_timestamp_ = meas_package.timestamp_;
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      float px = meas_package.raw_measurements_(0);
      float py = meas_package.raw_measurements_(1);
      x_ << px, py, 0, 0, 0;
    } else {
      float px = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
      float py = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
      float v = fabs(meas_package.raw_measurements_(2));
      float yaw = meas_package.raw_measurements_(1);
      float yawRate = 0;
      x_ << px, py, v, yaw, yawRate;
    }
    is_initialized_ = true;
    return;
  }
  
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; //dt in seconds
  // only update prior if a certain amount of time has passed
  if (dt > 0.0001) {
    previous_timestamp_ = meas_package.timestamp_;
    Prediction(dt);
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  } else {
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  //// CREATE SIGMA POINTS ////
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  Xsig.col(0) = x_;
  // calculate square root of P
  MatrixXd sqrt_P = P_.llt().matrixL();
  for (int i=0; i<n_x_; i++) {
    Xsig.col(i+1) = x_ + sqrt(lambda_ + n_x_) * sqrt_P.col(i);
    Xsig.col(n_x_+i+1) = x_ - sqrt(lambda_ + n_x_) * sqrt_P.col(i);
  }
  
  //// AUGMENT SIGMA POINTS ////
  // create process noise covariance matrix
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0, 0, std_yawdd_*std_yawdd_;
  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;
  MatrixXd sqrt_P_aug= P_aug.llt().matrixL();
  // create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  Xsig_aug.col(0) = x_aug;
  for (int i=0; i<n_aug_; i++) {
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*sqrt_P_aug.col(i);
      Xsig_aug.col(n_aug_+i+1) = x_aug - sqrt(lambda_+n_aug_)*sqrt_P_aug.col(i);
  }
  
  //// PREDICT SIGMA POINTS ////
  for (int i=0; i<2*n_aug_+1; i++) {
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawRate = Xsig_aug(4,i);
    double v_acc_noise = Xsig_aug(5,i);
    double yaw_acc_noise = Xsig_aug(6,i);
    double px_pred;
    double py_pred;
    double v_pred;
    double yaw_pred;
    double yawRate_pred;
    
    double delta_t_sq = delta_t*delta_t;
    v_pred = v + delta_t*v_acc_noise;
    yaw_pred = yaw + yawRate*delta_t;
    yawRate_pred = yawRate + delta_t*yaw_acc_noise;
    if (fabs(yawRate) < 0.001) {
      px_pred = px + (v*cos(yaw)*delta_t);
      py_pred = py + (v*sin(yaw)*delta_t);
    } else {
      px_pred = px + ((v/yawRate) * (sin(yaw+yawRate*delta_t)-sin(yaw)));
      py_pred = py + ((v/yawRate) * (-cos(yaw+yawRate*delta_t)+cos(yaw)));
    }
    
    // add noise
    px_pred  += 0.5*delta_t_sq*cos(yaw)*v_acc_noise;
    py_pred  += 0.5*delta_t_sq*sin(yaw)*v_acc_noise;
    yaw_pred += 0.5*delta_t_sq*yaw_acc_noise;
    
    Xsig_pred_(0,i) = px_pred;
    Xsig_pred_(1,i) = py_pred;
    Xsig_pred_(2,i) = v_pred;
    Xsig_pred_(3,i) = yaw_pred;
    Xsig_pred_(4,i) = yawRate_pred;
  }  
  
  //// PREDICT STATE MEAN ////
  x_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  //predict state covariance matrix
  P_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */  
  //// TRANSFORM SIGMA POINTS INTO MEASUREMENT SPACE ////
  int n_z = 3; // number of radar measurement elements
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  for (int i=0; i < 2*n_aug_+1; i++) {
    double px_pred = Xsig_pred_(0,i);
    double py_pred = Xsig_pred_(1,i);
    double v_pred = Xsig_pred_(2,i);
    double yaw_pred = Xsig_pred_(3,i);
    double yaw_pred_dot = Xsig_pred_(4,i);
    double p_pred_meas = sqrt(px_pred*px_pred + py_pred*py_pred);
    if (p_pred_meas < 0.00001) p_pred_meas = 0.00001;
    double rho_pred_meas = atan2(py_pred,px_pred);
    double p_dot_pre_meas = ((px_pred*cos(yaw_pred)*v_pred) + (py_pred*sin(yaw_pred)*v_pred)) / p_pred_meas;
    Zsig(0,i) = p_pred_meas;
    Zsig(1,i) = rho_pred_meas;
    Zsig(2,i) = p_dot_pre_meas;
  }
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  // calculate measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;
  S.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  S = S + R;
  
  //// UPDATE x and P FROM MEASUREMENT ////
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (x_diff(1)> M_PI) x_diff(1)-=2.*M_PI;
    while (x_diff(1)<-M_PI) x_diff(1)+=2.*M_PI;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    Tc = Tc + weights_(i)*x_diff*z_diff.transpose();
  }
  
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}
