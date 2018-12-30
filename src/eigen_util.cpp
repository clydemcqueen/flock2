#include "eigen_util.hpp"

#include <iostream>

#include "opencv2/calib3d/calib3d.hpp"

// Notes:
// -- OpenCV matrices are in row-major order, whereas Eigen matrices are column-major order
// -- ROS quaternions are x, y, z, w, whereas Eigen quaternions are w, x, y, z

namespace eigen_util {

Eigen::Affine3d to_affine(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
{
  Eigen::Vector3d t;
  t.x() = tvec.val[0];
  t.y() = tvec.val[1];
  t.z() = tvec.val[2];

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  Eigen::Matrix3d R;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      R(col, row) = rmat.at<double>(row, col);  // Row- vs. column-major order
    }
  }

  Eigen::Affine3d result;
  result.translation() = t;
  result.linear() = R;

  return result;
}

Eigen::Affine3d to_affine(const geometry_msgs::msg::Pose &pose)
{
  Eigen::Vector3d t;
  t.x() = pose.position.x;
  t.y() = pose.position.y;
  t.z() = pose.position.z;

  Eigen::Quaterniond q;
  q.x() = pose.orientation.x;
  q.y() = pose.orientation.y;
  q.z() = pose.orientation.z;
  q.w() = pose.orientation.w;

  Eigen::Affine3d result;
  result.translation() = t;
  result.linear() = q.toRotationMatrix();

  return result;
}

geometry_msgs::msg::Pose to_pose(const Eigen::Affine3d &transform)
{
  Eigen::Vector3d t = transform.translation();
  Eigen::Quaterniond q;
  q = transform.rotation();

  geometry_msgs::msg::Pose result;
  result.position.x = t.x();
  result.position.y = t.y();
  result.position.z = t.z();
  result.orientation.x = q.x();
  result.orientation.y = q.y();
  result.orientation.z = q.z();
  result.orientation.w = q.w();

#ifdef DEBUG
  assert(!is_nan(result));
#endif

  return result;
}

geometry_msgs::msg::Pose to_pose(double tx, double ty, double tz, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Pose result;
  result.position.x = tx;
  result.position.y = ty;
  result.position.z = tz;
  result.orientation.x = qx;
  result.orientation.y = qy;
  result.orientation.z = qz;
  result.orientation.w = qw;

#ifdef DEBUG
  assert(!is_nan(result));
#endif

  return result;
}

Eigen::Vector3d to_euler(const Eigen::Affine3d &transform)
{
  return transform.linear().eulerAngles(0, 1, 2);
}

Eigen::Vector3d to_euler(const geometry_msgs::msg::Pose pose)
{
  // w, x, y, z
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

bool is_nan(const geometry_msgs::msg::Pose p1)
{
  return std::isnan(p1.position.x) ||
    std::isnan(p1.position.y) ||
    std::isnan(p1.position.z) ||
    std::isnan(p1.orientation.x) ||
    std::isnan(p1.orientation.y) ||
    std::isnan(p1.orientation.z) ||
    std::isnan(p1.orientation.w);
}

geometry_msgs::msg::Transform to_tf(const Eigen::Affine3d &transform)
{
  Eigen::Quaterniond q(transform.linear());
  geometry_msgs::msg::Transform tf;
  tf.translation.x = transform.translation().x();
  tf.translation.y = transform.translation().y();
  tf.translation.z = transform.translation().z();
  tf.rotation.x = q.x();
  tf.rotation.y = q.y();
  tf.rotation.z = q.z();
  tf.rotation.w = q.w();
  return tf;
}

geometry_msgs::msg::Transform to_tf(const geometry_msgs::msg::Pose pose)
{
  geometry_msgs::msg::Transform tf;
  tf.translation.x = pose.position.x;
  tf.translation.y = pose.position.y;
  tf.translation.z = pose.position.z;
  tf.rotation.x = pose.orientation.x;
  tf.rotation.y = pose.orientation.y;
  tf.rotation.z = pose.orientation.z;
  tf.rotation.w = pose.orientation.w;
  return tf;
}

bool all_close(const geometry_msgs::msg::Pose p1, const geometry_msgs::msg::Pose p2, double e)
{
  return std::abs(p1.position.x - p2.position.x) < e &&
    std::abs(p1.position.y - p2.position.y) < e &&
    std::abs(p1.position.z - p2.position.z) < e &&
    std::abs(p1.orientation.x - p2.orientation.x) < e &&
    std::abs(p1.orientation.y - p2.orientation.y) < e &&
    std::abs(p1.orientation.z - p2.orientation.z) < e &&
    std::abs(p1.orientation.w - p2.orientation.w) < e;
}

std::ostream &operator<<(std::ostream &os, const Eigen::Affine3d &affine3d)
{
  os << "[";
  for (int row = 0; row < 4; row++) {
    os << "[";
    for (int col = 0; col < 4; col++) {
      os << affine3d.data()[col * 4 + row];
      if (col < 3) {
        os << ", ";
      }
    }
    os << "]";
    if (row < 3) {
      os << ", ";
    }
  }
  os << "]";
  return os;
}

void test1()
{
  geometry_msgs::msg::Pose p1 = to_pose(1, 2, 3, 0, 0, 1, 0);
  Eigen::Affine3d T1 = to_affine(p1);
  geometry_msgs::msg::Pose p2 = to_pose(T1);

  std::cout << "euler: " << to_euler(p1) << std::endl;
  std::cout << "euler: " << to_euler(T1) << std::endl;
  std::cout << "euler: " << to_euler(p2) << std::endl;

  if (is_nan(p1) || is_nan(p2) || !all_close(p1, p2)) {
    std::cout << "Test 1 failed" << std::endl;
  }
}

void test2()
{
  geometry_msgs::msg::Pose p0 = to_pose(0, 0, 0, 0, 0, 0, 1);
  geometry_msgs::msg::Pose p1 = to_pose(1, 2, 3, 0, 0, 0, 1);
  geometry_msgs::msg::Pose p2 = to_pose(-1, -2, -3, 0, 0, 0, 1);

  Eigen::Affine3d T1 = to_affine(p1);
  Eigen::Affine3d T2 = to_affine(p2);
  Eigen::Affine3d T3 = T1 * T2;
  geometry_msgs::msg::Pose p3 = to_pose(T3);

  if (is_nan(p3) || !all_close(p0, p3)) {
    std::cout << "Test 2 failed" << std::endl;
  }
}

void test3()
{
  geometry_msgs::msg::Pose p1 = to_pose(1, 2, 3, 0.5, -0.5, -0.5, 0.5);
  Eigen::Affine3d T2 = to_affine(p1).inverse(Eigen::TransformTraits::Isometry);
  geometry_msgs::msg::Pose p3 = to_pose(T2.inverse(Eigen::TransformTraits::Isometry));

  if (is_nan(p1) || is_nan(p3) || !all_close(p1, p3)) {
    std::cout << "Test 3 failed" << std::endl;
  }
}

void test_eigen()
{
  test1();
  test2();
  test3();
}

} // namespace eigen_util
