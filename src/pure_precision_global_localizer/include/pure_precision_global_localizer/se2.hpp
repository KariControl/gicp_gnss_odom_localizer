// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace pure_precision_global_localizer
{

constexpr double kPi = 3.14159265358979323846;

inline double wrapAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

inline double clampMagnitude(double value, double limit)
{
  if (!(limit >= 0.0) || !std::isfinite(limit)) {
    return 0.0;
  }
  return std::clamp(value, -limit, limit);
}

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  [[nodiscard]] bool finite() const
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
  }
};

inline Eigen::Matrix2d rotation2(double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix2d rotation;
  rotation << c, -s, s, c;
  return rotation;
}

inline Pose2 compose(const Pose2 & lhs, const Pose2 & rhs)
{
  const Eigen::Vector2d translation =
    Eigen::Vector2d(lhs.x, lhs.y) + rotation2(lhs.yaw) * Eigen::Vector2d(rhs.x, rhs.y);
  return {translation.x(), translation.y(), wrapAngle(lhs.yaw + rhs.yaw)};
}

inline Pose2 inverse(const Pose2 & pose)
{
  const Eigen::Vector2d translation =
    -rotation2(-pose.yaw) * Eigen::Vector2d(pose.x, pose.y);
  return {translation.x(), translation.y(), wrapAngle(-pose.yaw)};
}

inline Eigen::Vector2d transformPoint(const Pose2 & transform, const Eigen::Vector2d & point)
{
  return Eigen::Vector2d(transform.x, transform.y) + rotation2(transform.yaw) * point;
}

inline double poseTranslationDistance(const Pose2 & lhs, const Pose2 & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // namespace pure_precision_global_localizer
