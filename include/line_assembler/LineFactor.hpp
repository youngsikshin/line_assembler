#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <line_extractor/LineSegment.h>


template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T theta) {
  const T cos_theta = ceres::cos(theta);
  const T sin_theta = ceres::sin(theta);

  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_theta, -sin_theta, sin_theta, cos_theta;
  return rotation;
}

// Normalizes the angle in radians between [-pi and pi).
template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
      two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
  }
};


struct LineFactor
{
    LineFactor(line_extractor::LineSegment scan_line, line_extractor::LineSegment map_line)
        : sp(scan_line.sp().x(), scan_line.sp().y()), ep(scan_line.ep().x(), scan_line.ep().y())
    {
        map_param_theta = map_line.theta();
        map_param_rho = map_line.rho();
    }

    template <typename T>
    bool operator()(const T *x, const T *y, const T *theta, T *residual) const {
        Eigen::Matrix<T, 2, 1> t(*x, *y);
        // T_w_l
        Eigen::Matrix<T, 2, 2> rot = RotationMatrix2D(*theta);

        Eigen::Matrix<T, 2, 1> sp_w = rot * sp.cast<T>() + t;
        Eigen::Matrix<T, 2, 1> ep_w = rot * ep.cast<T>() + t;

        T nx = static_cast<T> (cosf(map_param_theta));
        T ny = static_cast<T> (sinf(map_param_theta));

        residual[0] = nx * sp_w(0) + ny * sp_w(1) - static_cast<T> (map_param_rho);
        residual[1] = nx * ep_w(0) + ny * ep_w(1) - static_cast<T> (map_param_rho);

        return true;
    }

    static ceres::CostFunction* Create(const line_extractor::LineSegment scan_line, const line_extractor::LineSegment map_line)
    {
        return (new ceres::AutoDiffCostFunction<LineFactor, 2, 1, 1, 1>(new LineFactor(scan_line, map_line)));
    }

    Eigen::Vector2f sp;
    Eigen::Vector2f ep;
    float map_param_theta;
    float map_param_rho;
};