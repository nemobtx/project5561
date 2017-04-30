/** @file vision.h
 *	@brief The vision.h file.
 *
 *	This file contains the computer vision functionalities in
 *	estimating the depth of points.
 *  Created on: Jan 16, 2014
 *  Author: Ahmed Medhat
 *  email: medhat@cs.umn.edu
 */

#ifndef VISION_H_
#define VISION_H_

#include <Eigen/Core>			/// To use: Matrix, Vector
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include "geometry/geometry.h"
#include <iostream>
#include <algebra.h>
#include <float.h>

//#include <tictoc.h>

/** @namespace Geometry
 * The Geometry namespace.
 */
namespace Geometry {

/// @fn EstimateDepth, Triangulation Function
Eigen::Vector2d EstimateDepth(const Eigen::Vector3d &p1,
                              const Eigen::Vector3d &p2,
                              const Eigen::Matrix3d &c2_R_c1,
                              const Eigen::Vector3d &c2_t_c1);

inline Eigen::Vector2d EstimateDepth(const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2,
                                     const Eigen::Vector4d &c2_q_c1,
                                     const Eigen::Vector3d &c2_t_c1) {
  return EstimateDepth(p1, p2, QuaternionToRotationMatrix(c2_q_c1), c2_t_c1);
}

/// @fn EstimateUnitTranslation, Estimate the unit translation vector between two camera poses.
/// i.e. position of pose 1 w.r.t. pose 2.
///	given two observed points (bearing vectors "unit homogeneous coordinates vector")
/// and the relative orientaion of pose 1 w.r.t. pose 2.
inline Eigen::Vector3d EstimateUnitTranslation(const Eigen::Vector3d &bearing11,
                                               const Eigen::Vector3d &bearing12,
                                               const Eigen::Vector3d &bearing21,
                                               const Eigen::Vector3d &bearing22,
                                               const Eigen::Matrix3d &c2_R_c1) {
  Eigen::Vector3d b = c2_R_c1 * bearing11;
  Eigen::Vector3d n1 = b.cross(bearing21);
  b = c2_R_c1 * bearing12;
  Eigen::Vector3d n2 = b.cross(bearing22);
  Eigen::Vector3d c2_t_c1 = n1.cross(n2);
  c2_t_c1 = c2_t_c1 / c2_t_c1.norm();
  return c2_t_c1;
}

/// Solve the five point algorithm, returns 1 on success 0 on failure.
int FivePointAlgorithm(const Eigen::Matrix<double, 3, 5> &p1,
                       const Eigen::Matrix<double, 3, 5> &p2,
                       Eigen::Matrix3d& c2_R_c1, Eigen::Vector3d& c2_t_c1);


/// Two-view 3D landmark mid-point triangulation.
template <typename Derived1, typename Derived2,
typename Derived3, typename Derived4, typename Derived5>
inline bool TwoViewMidPointTriangulation(const Eigen::MatrixBase<Derived1> & _b1,
                                         const Eigen::MatrixBase<Derived2> & _b2,
                                         const Eigen::MatrixBase<Derived3> & _C1_p_C2,
                                         const Eigen::MatrixBase<Derived4> & _C1_R_C2,
                                         const Eigen::MatrixBase<Derived5> & _C1_p_f,
                                         const typename Derived5::Scalar _tolerance = 0.005) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived1>, 3, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived2>, 3, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived3>, 3, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived4>, 3, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived5>, 3, 1);

  Eigen::MatrixBase<Derived5> & C1_p_f = const_cast<Eigen::MatrixBase<Derived5> &>(_C1_p_f);

  Eigen::Matrix<typename Derived5::Scalar, 3, 3> A;
  A.col(0) = _b1.normalized();
  A.col(1) = _C1_R_C2 * _b2.normalized();
  A.col(2) = A.col(0).cross(A.col(1));

  Eigen::Matrix<typename Derived5::Scalar, 3, 1> d = A.lu().solve(_C1_p_C2);
  C1_p_f = d(0)*A.col(0) + 0.5*d(2)*A.col(2);

  return (A.col(2).norm() >= _tolerance);   // for tolerance 0.005 -> ~0.3 deg.
}

/// Two-view 3D landmark LS triangulation.
#define TWO_VIEW_LS_TRIANGULATION_USE_ALGEBRA_LS_SOLVE 0 // Supported only for single precision.
template <typename Derived1, typename Derived2,
typename Derived3, typename Derived4, typename Derived5>
inline bool TwoViewLSTriangulation(const Eigen::MatrixBase<Derived1> & _b1,
                                   const Eigen::MatrixBase<Derived2> & _b2,
                                   const Eigen::MatrixBase<Derived3> & _C1_p_C2,
                                   const Eigen::MatrixBase<Derived4> & _C1_R_C2,
                                   const Eigen::MatrixBase<Derived5> & _C1_p_f,
                                   const typename Derived5::Scalar _tolerance = 0.005) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived1>, 3, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived2>, 3, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived3>, 3, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived4>, 3, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived5>, 3, 1);

  Eigen::MatrixBase<Derived5> & C1_p_f = const_cast<Eigen::MatrixBase<Derived5> &>(_C1_p_f);

  Eigen::Matrix<typename Derived5::Scalar, 3, 1> b1 = _b1.normalized();
  Eigen::Matrix<typename Derived5::Scalar, 3, 1> b2 = _C1_R_C2 * _b2.normalized();

  Eigen::Matrix<typename Derived5::Scalar, 4, 4,Eigen::RowMajor> A;
  A.row(0) << b1.cross(b2).transpose(), 0.0;
  A.row(1) << b1.cross(A.template block<1,3>(0,0)).transpose() , 0.0;
  A.row(2) << A.template block<1,3>(0,0), A.template block<1,3>(0,0) * _C1_p_C2;
  A.template block<1,3>(3,0) = b2.cross(A.template block<1,3>(0,0)).transpose();
  A(3,3) = A.template block<1,3>(3,0) * _C1_p_C2;

  typename Derived5::Scalar cross_bearing_norm = A.template block<1,3>(0,0).norm();

#if (TWO_VIEW_LS_TRIANGULATION_USE_ALGEBRA_LS_SOLVE == 1)
  algebra::LeastSquaresSolve3(A, C1_p_f.template segment<3>(0), FLT_MAX);
#else
  Eigen::HouseholderQR<Eigen::Matrix<typename Derived5::Scalar, 4,
                3, Eigen::RowMajor>> qr(A.template block<4,3>(0,0));
  C1_p_f = qr.solve(A.template block<4,1>(0,3));
#endif

  return (cross_bearing_norm > _tolerance); // for tolerance 0.005 -> ~0.3 deg.
}

/// Sequential LS Triangulation using Givens Transformations.
struct SequentialLSTriangulation {
 private:
  Eigen::Matrix<float, 3, 1> C1_p_f_;
  Eigen::Matrix<float, 3, 4, Eigen::RowMajor> R_;
  Eigen::Matrix<float, 3, 1> b1_;
 public:
  bool InitialTriangulation(const Eigen::Matrix<float, 3, 1> & _b1,
                            const Eigen::Matrix<float, 3, 1> & _b2,
                            const Eigen::Matrix<float, 3, 1> & _C1_p_C2,
                            const Eigen::Matrix<float, 3, 3> & _C1_R_C2,
                            const float _tolerance = 0.005) {
    b1_ = _b1.normalized();
    Eigen::Matrix<float, 3, 1> b2 = _C1_R_C2 * _b2.normalized();

    Eigen::Matrix<float, 4, 4,Eigen::RowMajor> A;
    A.row(0) << b1_.cross(b2).transpose(), 0.0;
    A.row(1) << b1_.cross(A.block<1,3>(0,0)).transpose() , 0.0;
    A.row(2) << A.block<1,3>(0,0), A.block<1,3>(0,0) * _C1_p_C2;
    A.block<1,3>(3,0) = b2.cross(A.block<1,3>(0,0)).transpose();
    A(3,3) = A.block<1,3>(3,0) * _C1_p_C2;

    float cross_bearing_norm = A.block<1,3>(0,0).norm();

    algebra::LeastSquaresSolve3(A, C1_p_f_.segment<3>(0), FLT_MAX);
    R_ = A.block<3,4>(0,0);

    return (cross_bearing_norm > _tolerance); // for tolerance 0.005 -> ~0.3 deg.
  }

  bool SequentialUpdateTriangulation(const Eigen::Matrix<float, 3, 1> & _bk,
                                     const Eigen::Matrix<float, 3, 1> & _C1_p_Ck,
                                     const Eigen::Matrix<float, 3, 3> & _C1_R_Ck,
                                     const float _tolerance = 0.005) {
    Eigen::Matrix<float, 3, 1> bk = _C1_R_Ck * _bk.normalized();

    Eigen::Matrix<float, 5, 4,Eigen::RowMajor> A;
    A.block<1,3>(0,0) << b1_.cross(bk).transpose();
    A(0,3) = A.block<1,3>(0,0) * _C1_p_Ck;
    A.block<1,3>(1,0) = bk.cross(A.block<1,3>(0,0)).transpose();
    A(3,3) = A.block<1,3>(1,0) * _C1_p_Ck;
    A.block<3,4>(2,0) = R_;

    float cross_bearing_norm = A.block<1,3>(0,0).norm();

    algebra::LeastSquaresSolve3(A, C1_p_f_.segment<3>(0), FLT_MAX);
    R_ = A.block<3,4>(0,0);

    return (cross_bearing_norm > _tolerance); // for tolerance 0.005 -> ~0.3 deg.
  }

  Eigen::Matrix<float, 3, 1> CurrentEstimate() {
    return C1_p_f_;
  }
};

/// Flags for solver types in triangulation function.
//#define LS_USE_QR
//#define LS_USE_SVD
#define LS_USE_NEON
//#define LS_USE_COLPIV_QR

/// Least-Squares feature position triangulation using bearing measurements from multiple camera poses.
template <typename Derived1, typename Derived2,
typename Derived3, typename Derived4, typename Derived5>
inline bool Triangulate(const Eigen::MatrixBase<Derived1> & _c1_p_ck,
                        const Eigen::MatrixBase<Derived2> & _c1_C_ck,
                        const Eigen::MatrixBase<Derived3> & _ck_b_f,
                        Eigen::MatrixBase<Derived4> const & _c1_p_f,
                        const Derived5 _tolerance = 250) {
  // Cast away the const-ness of _c1_p_f.
  Eigen::MatrixBase<Derived4> & c1_p_f_ = const_cast<Eigen::MatrixBase<Derived4> &>(_c1_p_f);
  // Create temporary matrices.
  Eigen::Matrix<typename Derived2::Scalar,
  Eigen::Dynamic, 4, Eigen::RowMajor> A(2 * _ck_b_f.cols(), 4);
  Eigen::Matrix<typename Derived2::Scalar, 3, 1> aj(3, 1);
  Eigen::Matrix<typename Derived2::Scalar, 3, 3> Aj(3, 3);
  for (unsigned int i = 0; i < _ck_b_f.cols(); i++) {
    aj = _c1_C_ck.template block<3, 3>(0, 3*i) *
        _ck_b_f.template block<3, 1>(0, i);
    Aj = Geometry::SkewSymmetricMatrix(aj);
    A.template block<2, 3>(2*i, 0) = Aj.template block<2, 3>(0, 0);
    A.template block<2, 1>(2*i, 3) = Aj.template block<2, 3>(0, 0) *
        _c1_p_ck.template block<3, 1>(0, i);
  }
#ifdef LS_USE_NEON
  // Solve the Least Squares problem.
  return algebra::LeastSquaresSolve3(A, c1_p_f_.template segment<3>(0), _tolerance);
#endif
#ifdef LS_USE_QR
  // Perform QR decomposition on A.
  Eigen::HouseholderQR<Eigen::Matrix<typename Derived2::Scalar, Eigen::Dynamic,
  3, Eigen::RowMajor>> qr(A.template block<Eigen::Dynamic, 3>(0, 0, 2 * _ck_b_f.cols(), 3));
  // Compute the condition number and compare it with the tolerance.
  Eigen::Matrix<typename Derived2::Scalar, 3, 1> R_diagonal_abs =
      qr.matrixQR().block(0, 0, 3, 3).diagonal().cwiseAbs();
  typename Derived2::Scalar condition_number = (R_diagonal_abs.minCoeff() != 0)?
        R_diagonal_abs.maxCoeff()/R_diagonal_abs.minCoeff() : INFINITY;
  if(condition_number > _tolerance) {
    return false;
  }
  // Solve the Least squares problem.
  c1_p_f_ = qr.solve(A.template block<Eigen::Dynamic, 1>(0, 3, 2 * _ck_b_f.cols(), 1));
  return true;
#endif
#ifdef LS_USE_SVD
  // Perform QR decomposition on A.
  Eigen::JacobiSVD<Eigen::Matrix<typename Derived2::Scalar, Eigen::Dynamic,
  Eigen::Dynamic, Eigen::RowMajor>> svd(A.template block<Eigen::Dynamic, 3>(0, 0, 2 * _ck_b_f.cols(), 3),
                           Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Compute the condition number and compare it with the tolerance.
  Eigen::Matrix<typename Derived2::Scalar, 3, 1> singular_values = svd.singularValues();
  typename Derived2::Scalar condition_number = (singular_values(singular_values.rows() - 1) != 0)?
      singular_values(0) / singular_values(singular_values.rows() - 1) : INFINITY;
  if(condition_number > _tolerance) {
    return false;
  }
  // Solve the Least squares problem.
  c1_p_f_ = svd.solve(A.template block<Eigen::Dynamic, 1>(0, 3, 2 * _ck_b_f.cols(), 1));
  return true;
#endif
#ifdef LS_USE_COLPIV_QR
  // Perform QR decomposition on A.
  Eigen::ColPivHouseholderQR<Eigen::Matrix<typename Derived2::Scalar, Eigen::Dynamic,
  3, Eigen::RowMajor>> qr(A.template block<Eigen::Dynamic, 3>(0, 0, 2 * _ck_b_f.cols(), 3));
  // Compute the condition number and compare it with the tolerance.
  Eigen::Matrix<typename Derived2::Scalar, 3, 1> R_diagonal_abs =
      qr.matrixQR().block(0, 0, 3, 3).diagonal().cwiseAbs();
  typename Derived2::Scalar condition_number = (R_diagonal_abs.minCoeff() != 0)?
        R_diagonal_abs.maxCoeff()/R_diagonal_abs.minCoeff() : INFINITY;
  if(condition_number > _tolerance) {
    return false;
  }
  // Solve the Least squares problem.
  c1_p_f_ = qr.solve(A.template block<Eigen::Dynamic, 1>(0, 3, 2 * _ck_b_f.cols(), 1));
  return true;
#endif
}

/// Least-Squares feature inverse depth (u,v,rho) triangulation using bearing measurements from multiple camera poses.
template <typename Derived1, typename Derived2,
typename Derived3, typename Derived4, typename Derived5>
inline bool TriangulateInvdep(const Eigen::MatrixBase<Derived1> & _c1_p_ck,
                              const Eigen::MatrixBase<Derived2> & _c1_C_ck,
                              const Eigen::MatrixBase<Derived3> & _ck_b_f,
                              Eigen::MatrixBase<Derived4> const & _c1_f_uvrho,
                              bool _check_condition, const Derived5 & _tolerance = 50) {
  // Cast away the const-ness of _c1_f_uvrho.
  (void)_check_condition;
  Eigen::MatrixBase<Derived4> & c1_f_uvrho_ = const_cast<Eigen::MatrixBase<Derived4> &>(_c1_f_uvrho);
  // Create temporary matrices.
  Eigen::Matrix<typename Derived2::Scalar, Eigen::Dynamic, 4, Eigen::RowMajor> A(2 * _ck_b_f.cols(), 4);
  Eigen::Matrix<typename Derived2::Scalar, 3, 1> aj(3, 1);
  Eigen::Matrix<typename Derived2::Scalar, 3, 3, Eigen::RowMajor> Aj(3, 3);
  for (unsigned int i = 0; i < _ck_b_f.cols(); i++) {
    aj = _c1_C_ck.template block<3, 3>(0, 3*i) *
        _ck_b_f.template block<3, 1>(0, i);
    Aj = Geometry::SkewSymmetricMatrix(aj);
    A.template block<2, 2>(2*i, 0) = Aj.template block<2, 2>(0, 0);
    A.template block<2, 1>(2*i, 2) = - Aj.template block<2, 3>(0, 0) *
        _c1_p_ck.template block<3, 1>(0, i);
    A.template block<2, 1>(2*i, 3) = - Aj.template block<2, 1>(0, 2);
  }
#ifdef LS_USE_NEON
  // Solve the Least Squares problem.
  return algebra::LeastSquaresSolve3(A, c1_f_uvrho_.template segment<3>(0), _tolerance);
#endif
#ifdef LS_USE_SVD
  // Perform SVD on A.
  Eigen::JacobiSVD<Eigen::Matrix<typename Derived2::Scalar, Eigen::Dynamic, Eigen::Dynamic>> svd(
      A.template block<Eigen::Dynamic, 3>(0, 0, 2 * _ck_b_f.cols(), 3),
      Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (_check_condition) {
    // Compute the condition number and compare it with the tolerance.
    Eigen::Matrix<typename Derived2::Scalar, 3, 1> singular_values = svd.singularValues();
    typename Derived2::Scalar condition_number =
        (singular_values(2) != 0) ? singular_values(0) / singular_values(2) : INFINITY;
    if (condition_number > _tolerance) {
      return false;
    }
  }
  // Solve the Least squares problem.
  c1_f_uvrho_ = svd.solve(A.template block<Eigen::Dynamic, 1>(0, 3, 2 * _ck_b_f.cols(), 1));
  return true;
#endif
}

} /** End of namespace: Geometry */
#endif /** End of file: VISION_H_ */
