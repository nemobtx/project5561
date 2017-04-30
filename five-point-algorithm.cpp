/** @file five-point-algorithm.cpp
 *	@brief The five-point-algorithm.cpp file.
 *
 *	This file contains
 *  Created on: Jan 16, 2014
 *  Author: Ahmed Medhat
 *  email: medhat@cs.umn.edu
 */

#include "geometry/vision.h"
#include "geometry/error.h"

#include <math.h>       /// to use: fabs
#include <float.h>		/// To use: DBL_MAX
#include <Eigen/Eigenvalues>

/** @namespace Geometry
 * The Geometry namespace.
 */
namespace Geometry {

/* Nister's Five point algorithm simple implementation.
 As implemented in the Matlab code at his website and
 at the implementation in the src code of the libmv project.*/

enum {
  coef_xxx,
  coef_xxy,
  coef_xyy,
  coef_yyy,
  coef_xxz,
  coef_xyz,
  coef_yyz,
  coef_xzz,
  coef_yzz,
  coef_zzz,
  coef_xx,
  coef_xy,
  coef_yy,
  coef_xz,
  coef_yz,
  coef_zz,
  coef_x,
  coef_y,
  coef_z,
  coef_1
};
inline Eigen::VectorXd o1(const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
  Eigen::Matrix<double, 20, 1> res = Eigen::Matrix<double, 20, 1>::Zero();
  res(coef_xx) = a(coef_x) * b(coef_x);
  res(coef_xy) = a(coef_x) * b(coef_y) + a(coef_y) * b(coef_x);
  res(coef_xz) = a(coef_x) * b(coef_z) + a(coef_z) * b(coef_x);
  res(coef_yy) = a(coef_y) * b(coef_y);
  res(coef_yz) = a(coef_y) * b(coef_z) + a(coef_z) * b(coef_y);
  res(coef_zz) = a(coef_z) * b(coef_z);
  res(coef_x) = a(coef_x) * b(coef_1) + a(coef_1) * b(coef_x);
  res(coef_y) = a(coef_y) * b(coef_1) + a(coef_1) * b(coef_y);
  res(coef_z) = a(coef_z) * b(coef_1) + a(coef_1) * b(coef_z);
  res(coef_1) = a(coef_1) * b(coef_1);

  return res;
}

inline Eigen::VectorXd o2(const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
  Eigen::Matrix<double, 20, 1> res;
  res(coef_xxx) = a(coef_xx) * b(coef_x);
  res(coef_xxy) = a(coef_xx) * b(coef_y) + a(coef_xy) * b(coef_x);
  res(coef_xxz) = a(coef_xx) * b(coef_z) + a(coef_xz) * b(coef_x);
  res(coef_xyy) = a(coef_xy) * b(coef_y) + a(coef_yy) * b(coef_x);
  res(coef_xyz) = a(coef_xy) * b(coef_z) + a(coef_yz) * b(coef_x)
      + a(coef_xz) * b(coef_y);
  res(coef_xzz) = a(coef_xz) * b(coef_z) + a(coef_zz) * b(coef_x);
  res(coef_yyy) = a(coef_yy) * b(coef_y);
  res(coef_yyz) = a(coef_yy) * b(coef_z) + a(coef_yz) * b(coef_y);
  res(coef_yzz) = a(coef_yz) * b(coef_z) + a(coef_zz) * b(coef_y);
  res(coef_zzz) = a(coef_zz) * b(coef_z);
  res(coef_xx) = a(coef_xx) * b(coef_1) + a(coef_x) * b(coef_x);
  res(coef_xy) = a(coef_xy) * b(coef_1) + a(coef_x) * b(coef_y)
      + a(coef_y) * b(coef_x);
  res(coef_xz) = a(coef_xz) * b(coef_1) + a(coef_x) * b(coef_z)
      + a(coef_z) * b(coef_x);
  res(coef_yy) = a(coef_yy) * b(coef_1) + a(coef_y) * b(coef_y);
  res(coef_yz) = a(coef_yz) * b(coef_1) + a(coef_y) * b(coef_z)
      + a(coef_z) * b(coef_y);
  res(coef_zz) = a(coef_zz) * b(coef_1) + a(coef_z) * b(coef_z);
  res(coef_x) = a(coef_x) * b(coef_1) + a(coef_1) * b(coef_x);
  res(coef_y) = a(coef_y) * b(coef_1) + a(coef_1) * b(coef_y);
  res(coef_z) = a(coef_z) * b(coef_1) + a(coef_1) * b(coef_z);
  res(coef_1) = a(coef_1) * b(coef_1);
  return res;
}

void EstimateEssentialMatrix5Point(const Eigen::Matrix<double, 3, 5> &p1,
                                   const Eigen::Matrix<double, 3, 5> &p2,
                                   Eigen::Matrix<double, 3, 30>* essential,
                                   int& numOfSols) {
  numOfSols = 0;
  assert(p1.cols() == 5);
  assert(p2.cols() == 5);
  Eigen::Matrix<double, 9, 9> constraint;
  constraint.setZero();
  for (int i = 0; i < 5; i++) {
    constraint(i, 0) = p2(0, i) * p1(0, i);
    constraint(i, 1) = p2(0, i) * p1(1, i);
    constraint(i, 2) = p2(0, i);
    constraint(i, 3) = p2(1, i) * p1(0, i);
    constraint(i, 4) = p2(1, i) * p1(1, i);
    constraint(i, 5) = p2(1, i);
    constraint(i, 6) = p1(0, i);
    constraint(i, 7) = p1(1, i);
    constraint(i, 8) = 1.0;
  }
  Eigen::Matrix<double, 9, 4> nullspaceOfConstraint;
  nullspaceOfConstraint.setZero();

  //nullspaceOfConstraint = constraint.svd().matrixV().block(0,5,9,6);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      constraint, Eigen::ComputeFullV | Eigen::ComputeFullU);
  nullspaceOfConstraint = svd.matrixV().topRightCorner<9, 4>();

  Eigen::VectorXd E[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      E[i][j] = Eigen::VectorXd::Zero(20);
      E[i][j](coef_x) = nullspaceOfConstraint(3 * i + j, 0);
      E[i][j](coef_y) = nullspaceOfConstraint(3 * i + j, 1);
      E[i][j](coef_z) = nullspaceOfConstraint(3 * i + j, 2);
      E[i][j](coef_1) = nullspaceOfConstraint(3 * i + j, 3);
    }
  }

  // The constraint matrix.
  Eigen::Matrix<double, 10, 20> M;
  M.setZero();
  int mrow = 0;

  // Determinant constraint det(E) = 0; equation (19) of Nister [2].
  M.row(mrow++) = o2(o1(E[0][1], E[1][2]) - o1(E[0][2], E[1][1]), E[2][0])
      + o2(o1(E[0][2], E[1][0]) - o1(E[0][0], E[1][2]), E[2][1])
      + o2(o1(E[0][0], E[1][1]) - o1(E[0][1], E[1][0]), E[2][2]);

  // Cubic singular values constraint.
  // Equation (20).
  Eigen::VectorXd EET[3][3];
  for (int i = 0; i < 3; ++i) {    // Since EET is symmetric, we only compute
    for (int j = 0; j < 3; ++j) {  // its upper triangular part.
      EET[i][j].setZero();
      if (i <= j) {
        EET[i][j] = o1(E[i][0], E[j][0]) + o1(E[i][1], E[j][1])
            + o1(E[i][2], E[j][2]);
      } else {
        EET[i][j] = EET[j][i];
      }
    }
  }

  // Equation (21).
  Eigen::VectorXd (&L)[3][3] = EET;
  Eigen::VectorXd trace = 0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
  for (int i = 0; i < 3; ++i) {
    L[i][i] -= trace;
  }

  // Equation (23).
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Eigen::VectorXd LEij = o2(L[i][0], E[0][j]) + o2(L[i][1], E[1][j])
          + o2(L[i][2], E[2][j]);
      M.row(mrow++) = LEij;
    }
  }

  /*Gauss Jordan*/

  for (int i = 0; i < 10; ++i) {
    M.row(i) /= M(i, i);
    for (int j = i + 1; j < 10; ++j) {
      M.row(j) = M.row(j) / M(j, i) - M.row(i);
    }
  }

  // Back substitution.
  for (int i = 9; i >= 0; --i) {
    for (int j = 0; j < i; ++j) {
      M.row(j) = M.row(j) - M(j, i) * M.row(i);
    }
  }

  Eigen::MatrixXd B = M.topRightCorner<10, 10>();
  Eigen::Matrix<double, 10, 10> At;
  At.setZero();
  At.row(0) = -B.row(0);
  At.row(1) = -B.row(1);
  At.row(2) = -B.row(2);
  At.row(3) = -B.row(4);
  At.row(4) = -B.row(5);
  At.row(5) = -B.row(7);
  At(6, 0) = 1;
  At(7, 1) = 1;
  At(8, 3) = 1;
  At(9, 6) = 1;

  // Compute solutions from action matrix's eigenvectors.
  Eigen::EigenSolver<Eigen::Matrix<double, 10, 10> > es(At);
  Eigen::MatrixXcd V = es.eigenvectors();
  Eigen::MatrixXcd SOLS(4, 10);
  SOLS.row(0) = V.row(6).array() / V.row(9).array();
  SOLS.row(1) = V.row(7).array() / V.row(9).array();
  SOLS.row(2) = V.row(8).array() / V.row(9).array();
  SOLS.row(3).setOnes();

  // Get the ten candidate E matrices in vector form.
  Eigen::MatrixXcd nullspaceOfConstraint2(9, 4);
  nullspaceOfConstraint2.imag().setZero();
  nullspaceOfConstraint2.real() = nullspaceOfConstraint;
  Eigen::MatrixXcd Evec = nullspaceOfConstraint2 * SOLS;

  // Build essential matrices for the real solutions.
  for (int s = 0; s < 10; ++s) {
    Evec.col(s) /= Evec.col(s).norm();
    bool is_real = true;
    for (int i = 0; i < 9; ++i) {
      double compare_value = (Evec.imag())(i, s);
      if (fabs(compare_value) >= 1e-2) {
        is_real = false;
        break;
      }
    }
    if (is_real) {
      Eigen::Matrix3d E;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          E(i, j) = (Evec.real())(3 * i + j, s);
        }
      }
      (*essential).block(0, numOfSols * 3, 3, 3) = E;
      numOfSols++;
    }

  }

}

int CheckEstimatedRotation(Eigen::Matrix3d& essentialN, Eigen::Vector3d& point1,
                           Eigen::Vector3d& point2,
                           Eigen::Matrix3d& solvedRotation,
                           Eigen::Vector3d& translation) {
  Eigen::JacobiSVD<Eigen::Matrix3d> USV(
      essentialN, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix3d U = USV.matrixU();
  Eigen::Matrix3d Vt = USV.matrixV().transpose();

  // Last column of U is undetermined since d = (a a 0).
  if (U.determinant() < 0) {
    U.col(2) *= -1;
  }
  // Last row of Vt is undetermined since d = (a a 0).
  if (Vt.determinant() < 0) {
    Vt.row(2) *= -1;
  }
  Eigen::Matrix3d W;
  W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Vector3d trans = U.col(2);
  Eigen::Vector3d trans2 = (-1) * trans;
  Eigen::Vector2d testDepths;

  Eigen::Matrix3d rotation_matrix = U * W * Vt;
  testDepths = Geometry::EstimateDepth(point1, point2, rotation_matrix, trans);
  if ((testDepths(0) > 0) && (testDepths(1) > 0)) {
    solvedRotation = rotation_matrix;
    translation = trans;
    return 1;
  }
  testDepths = Geometry::EstimateDepth(point1, point2, rotation_matrix, trans2);
  if ((testDepths(0) > 0) && (testDepths(1) > 0)) {
    solvedRotation = rotation_matrix;
    translation = trans2;
    return 1;
  }

  rotation_matrix = U * W.transpose() * Vt;
  testDepths = Geometry::EstimateDepth(point1, point2, rotation_matrix, trans2);
  if ((testDepths(0) > 0) && (testDepths(1) > 0)) {
    solvedRotation = rotation_matrix;
    translation = trans2;
    return 1;
  }
  testDepths = Geometry::EstimateDepth(point1, point2, rotation_matrix, trans);
  if ((testDepths(0) > 0) && (testDepths(1) > 0)) {
    solvedRotation = rotation_matrix;
    translation = trans;
    return 1;
  }
  return 0;
}

/// Solve the five point algorithm
int FivePointAlgorithm(const Eigen::Matrix<double, 3, 5> &p1,
                       const Eigen::Matrix<double, 3, 5> &p2,
                       Eigen::Matrix3d& c2_R_c1, Eigen::Vector3d& c2_t_c1) {
  Eigen::Matrix<double, 3, 30> essentialFive = Eigen::Matrix<double, 3, 30>::Zero();
  Eigen::Matrix3d essential = Eigen::Matrix3d::Zero();
  int sols = 0;
  EstimateEssentialMatrix5Point(p1, p2, &essentialFive, sols);
  if (sols == 1) {
    essential = essentialFive.block(0, 0, 3, 3);
  } else {
    double min_error = DBL_MAX;
    int select = 0;
    for (int i = 0; i < sols; i++) {
      Eigen::Vector3d point1 = p1.col(4);
      Eigen::Vector3d point2 = p2.col(4);
      Eigen::Matrix3d essential_matrix = essentialFive.block(0, 3 * i, 3, 3);
      double error = Geometry::SampsonEpipolarError(point1, point2,
                                                    essential_matrix);
      error *= error;
      if (error < min_error) {
        select = i;
        min_error = error;
      }
    }
    essential = essentialFive.block(0, 3 * select, 3, 3);
  }
  Eigen::Vector3d test_point1 = p1.col(1);
  Eigen::Vector3d test_point2 = p2.col(1);
  return CheckEstimatedRotation(essential, test_point1, test_point2, c2_R_c1,
                                c2_t_c1);
}
} /** End of namespace: Geometry */
