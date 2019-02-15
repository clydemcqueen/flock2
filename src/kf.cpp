#include "kf.hpp"

#include <cassert>

namespace kf {

KalmanFilter::KalmanFilter(int state_dim, int measurement_dim) :
  state_dim_{state_dim},
  measurement_dim_{measurement_dim}
{
  assert(state_dim > 0);
  assert(measurement_dim > 0);

  x_ = Eigen::MatrixXd::Zero(state_dim, 1);
  P_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  Q_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  F_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  H_ = Eigen::MatrixXd::Zero(measurement_dim, state_dim);
  K_ = Eigen::MatrixXd::Zero(state_dim, measurement_dim);
  y_ = Eigen::MatrixXd::Zero(measurement_dim, 1);
  S_ = Eigen::MatrixXd::Zero(measurement_dim, measurement_dim);
}

void KalmanFilter::set_x(const Eigen::MatrixXd &x)
{
  assert(x.rows() == state_dim_ && x.cols() == 1);
  x_ = x;
}

void KalmanFilter::set_P(const Eigen::MatrixXd &P)
{
  assert(P.rows() == state_dim_ && P.cols() == state_dim_);
  P_ = P;
}

void KalmanFilter::set_Q(const Eigen::MatrixXd &Q)
{
  assert(Q.rows() == state_dim_ && Q.cols() == state_dim_);
  Q_ = Q;
}

void KalmanFilter::set_F(const Eigen::MatrixXd &F)
{
  assert(F.rows() == state_dim_ && F.cols() == state_dim_);
  F_ = F;
}

void KalmanFilter::set_H(const Eigen::MatrixXd &H)
{
  assert(H.rows() == measurement_dim_ && H.cols() == state_dim_);
  H_ = H;
}

void KalmanFilter::set_K(const Eigen::MatrixXd &K)
{
  assert(K.rows() == state_dim_ && K.cols() == measurement_dim_);
  K_ = K;
}

void KalmanFilter::set_y(const Eigen::MatrixXd &y)
{
  assert(y.rows() == measurement_dim_ && y.cols() == 1);
  y_ = y;
}

void KalmanFilter::set_S(const Eigen::MatrixXd &S)
{
  assert(S.rows() == measurement_dim_ && S.cols() == measurement_dim_);
  S_ = S;
}

void KalmanFilter::predict()
{
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

#if 1
  assert(x_.rows() == state_dim_ && x_.cols() == 1);
  assert(P_.rows() == state_dim_ && P_.cols() == state_dim_);
#endif
}

void KalmanFilter::update(const Eigen::MatrixXd &z, const Eigen::MatrixXd &R)
{
  assert(z.rows() == measurement_dim_ && z.cols() == 1);
  assert(R.rows() == measurement_dim_ && R.cols() == measurement_dim_);

  Eigen::MatrixXd PHT = P_ * H_.transpose();
  S_ = H_ * PHT + R;
  K_ = PHT * S_.inverse();
  y_ = z - H_ * x_;
  x_ = x_ + K_ * y_;
  P_ = P_ - K_ * H_ * P_;  // Should be P = (I - KH)P, but might be unstable?

#if 1
  assert(x_.rows() == state_dim_ && x_.cols() == 1);
  assert(P_.rows() == state_dim_ && P_.cols() == state_dim_);
#endif
}

} // namespace kf