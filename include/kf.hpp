#ifndef KF_H
#define KF_H

#include "eigen3/Eigen/Dense"

namespace kf {

class KalmanFilter
{
  int state_dim_;
  int measurement_dim_;

  Eigen::MatrixXd x_;   // State mean
  Eigen::MatrixXd P_;   // State covariance
  Eigen::MatrixXd Q_;   // Process covariance
  Eigen::MatrixXd F_;   // State transition function
  Eigen::MatrixXd H_;   // Measurement function
  Eigen::MatrixXd K_;   // Kalman gain
  Eigen::MatrixXd y_;   // Residual
  Eigen::MatrixXd S_;   // System uncertainty

public:
  explicit KalmanFilter(int state_dim, int measurement_dim);

  ~KalmanFilter() {}

  const auto &x() const { return x_; }
  const auto &P() const { return P_; }
  const auto &Q() const { return Q_; }
  const auto &F() const { return F_; }
  const auto &H() const { return H_; }
  const auto &K() const { return K_; }
  const auto &y() const { return y_; }
  const auto &S() const { return S_; }

  void set_x(const Eigen::MatrixXd &x);
  void set_P(const Eigen::MatrixXd &P);
  void set_Q(const Eigen::MatrixXd &Q);
  void set_F(const Eigen::MatrixXd &F);
  void set_H(const Eigen::MatrixXd &H);
  void set_K(const Eigen::MatrixXd &K);
  void set_y(const Eigen::MatrixXd &y);
  void set_S(const Eigen::MatrixXd &S);

  void predict();
  void update(const Eigen::MatrixXd &z, const Eigen::MatrixXd &R);
};

} // namespace kf

#endif // KF_H