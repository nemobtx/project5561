/** @file error.h
 *	@brief The error.h file.
 *
 *	This file contains the computer vision functionalities to
 *	calculate the error (i.e. different reprojection error functions).
 *  Created on: Jan 16, 2014
 *  Author: Ahmed Medhat
 *  email: medhat@cs.umn.edu
 */

#ifndef GEOMETRY_ERROR_H_
#define GEOMETRY_ERROR_H_

#include <Eigen/Core>         /// To use: Matrix, Vector
#include <cmath>              /// To use: std::fabs, std::isnan
#include <limits>             /// To use: std::numeric_limits
#include "geometry/vision.h"  /// To use: EstimateDepth


 #include "iks/dimitri_tools.h"


/** @namespace Geometry
 * The Geometry namespace.
 */
namespace Geometry {

inline double ReprojectionError(const Eigen::Vector3d &p1,
                                const Eigen::Vector3d &p2,
                                const Eigen::Matrix3d &rotation_2C1,
                                const Eigen::Vector3d &translation_2t1) {
  Eigen::Vector2d depths;
  depths = EstimateDepth(p1, p2, rotation_2C1, translation_2t1);
  Eigen::Vector3d t = translation_2t1 / translation_2t1.norm();
  ///t = t / fabs(max(t(2), max(t(1), t(0))));

  Eigen::MatrixXd p3d(3, 2);
  p3d.col(0) = (p1 / p1.norm()) * depths(0);
  p3d.col(1) = (p2 / p2.norm()) * depths(1);
  //Move it to camera2.
  p3d.col(0) = (rotation_2C1 * p3d.col(0)) + t;
  p3d.col(1) = (rotation_2C1.transpose() * p3d.col(1))
      - rotation_2C1.transpose() * t;
  //Project
  p3d.col(0) = p3d.col(0) / p3d(2, 0);
  p3d.col(1) = p3d.col(1) / p3d(2, 1);
  double error;
  error = (p3d.col(0) - p2).norm() + (p3d.col(1) - p1).norm();

  if (!std::isnan(error))
    return std::fabs(error);
  else
    return std::numeric_limits<double>::max();
}
inline double ReprojectionError(const Eigen::Vector3d &p1,
                                const Eigen::Vector3d &p2,
                                const Eigen::Vector4d &q_2C1,
                                const Eigen::Vector3d &translation_2t1) {
  return ReprojectionError(p1, p2, QuaternionToRotationMatrix(q_2C1),
                           translation_2t1);
}

/**
 * Epipolar Errors
 */
/// @fn AlgebraicEpipolarError, Calculate algebraic Epipolar error.
inline double AlgebraicEpipolarError(const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2,
                                     const Eigen::Matrix3d &essentialMatrix) {
  double error = p2.transpose() * essentialMatrix * p1;
  return error * error;
}
/// @fn SampsonEpipolarError, Calculate Sampson Epipolar error.
inline double SampsonEpipolarError(const Eigen::Vector3d &p1,
                                   const Eigen::Vector3d &p2,
                                   const Eigen::Matrix3d &essentialMatrix) {
  Eigen::Vector3d Fx1;
  Eigen::Vector3d Fx2;
  Fx1 = essentialMatrix * p1;
  Fx2 = essentialMatrix.transpose() * p2;
  double error, temp;
  temp = Fx1(0) * Fx1(0);
  temp += Fx1(1) * Fx1(1);
  temp += Fx2(1) * Fx2(1);
  temp += Fx2(0) * Fx2(0);
  /// error  = p2' * essentialMatrix * p1
  error = p2.transpose() * Fx1;
  error = (error * error) / temp;
  return error;
}

inline double AngleBetweenTwoVectors(const Eigen::Vector3d &vector_one,
                                     const Eigen::Vector3d &vector_two) {
  Eigen::Vector3d vector_alpha = vector_one / vector_one.norm();
  Eigen::Vector3d vector_beta = vector_two / vector_two.norm();
  Eigen::Vector3d cross = vector_alpha.cross(vector_beta);
  double sine = cross.norm();
  double cosine = vector_alpha.transpose() * vector_beta;
  double angle = RAD_TO_DEG * atan2(sine, cosine);
  return angle;
}

inline double AngleError(const Eigen::Vector3d &b1, const Eigen::Vector3d &b2) {
  double cosine = b1.transpose() * b2;
  double sine = (Geometry::SkewSymmetricMatrix(b1)*b2).norm();
  // if((RAD_TO_DEG * atan2(sine,cosine)) > 10.0) {
  //   debug_matrix(sine);
  //   debug_matrix(cosine);
  //   debug_matrix(b1);
  //   debug_matrix(b2);
  //   STOP_HERE;
  // }
  double angle = RAD_TO_DEG * atan2(sine,cosine);
  // debug_matrix(b1.transpose());
  // debug_matrix(b2.transpose());
    // debug_matrix(angle);

  return angle;
}
} /** End of namespace: Geometry */
#endif /** End of file: GEOMETRY_ERROR_H_ */
