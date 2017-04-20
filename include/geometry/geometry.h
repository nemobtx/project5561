// Templated version of numerics.h
#ifndef GEOMETRY_H_
#define GEOMETRY_H_

// Since there is a conflicting definition in X11.h, we undefine Success here.
#undef Success
#include <Eigen/Core>
/** @namespace Geometry
 * The Geometry namespace.
 */
namespace Geometry {

const double PI = 3.14159265359;
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;
const double USEC_PER_SEC = 1.e6;
const double MSEC_PER_SEC = 1.e3;

template<typename scalar>
inline scalar AngleBetweenTwoNormalVectors(
    Eigen::Matrix<scalar, 3, 1> &vector_1,
    Eigen::Matrix<scalar, 3, 1> &vector_2) {
  scalar cosine = vector_1.dot(vector_2);
  Eigen::Matrix<scalar, 3, 3> s;
  scalar zero = 0;
  s << zero, -vector_1(2), vector_1(1), vector_1(2), zero, -vector_1(0), -vector_1(
      1), vector_1(0), zero;
  scalar sine = (s * vector_2).norm();
  return RAD_TO_DEG * atan2(sine, cosine);
}

template<typename scalar>
inline scalar AngleBetweenTwoVectors(Eigen::Matrix<scalar, 3, 1> &vector_1,
                                     Eigen::Matrix<scalar, 3, 1> &vector_2) {
  scalar cosine = vector_1.dot(vector_2);
  Eigen::Matrix<scalar, 3, 3> s;
  scalar zero = 0;
  s << zero, -vector_1(2), vector_1(1), vector_1(2), zero, -vector_1(0), -vector_1(
      1), vector_1(0), zero;
  scalar sine = (s * vector_2).norm();
  return RAD_TO_DEG
      * atan2(sine / (vector_1.norm() * vector_2.norm()),
              cosine / vector_1.norm() * vector_2.norm());
}

// Omega matrix used in the computation of the quaternion time derivative.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Omega(
    const Eigen::MatrixBase<Derived> &w) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 1);
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> omega;
  omega << zero, w(2), -w(1), w(0), -w(2), zero, w(0), w(1), w(1), -w(0), zero, w(
      2), -w(0), -w(1), -w(2), zero;
  return omega;
}

// Quaternion inverse.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> QuaternionInverse(
    const Eigen::MatrixBase<Derived> &q) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 4, 1> inverse_q = q;
  inverse_q.template block<3, 1>(0, 0) = -q.template block<3, 1>(0, 0);
  return inverse_q;
}

// Conversion from quaternion to rotation matrix.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> QuaternionToRotationMatrix(
    const Eigen::MatrixBase<Derived> &q) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
  typename Derived::Scalar two = static_cast<typename Derived::Scalar>(2);
  R(0, 0) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  R(0, 1) = two * (q(0) * q(1) + q(2) * q(3));
  R(0, 2) = two * (q(0) * q(2) - q(1) * q(3));
  R(1, 0) = two * (q(0) * q(1) - q(2) * q(3));
  R(1, 1) = -q(0) * q(0) + q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  R(1, 2) = two * (q(1) * q(2) + q(0) * q(3));
  R(2, 0) = two * (q(0) * q(2) + q(1) * q(3));
  R(2, 1) = two * (q(1) * q(2) - q(0) * q(3));
  R(2, 2) = -q(0) * q(0) - q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
  return R;
}

// Conversion from rotation matrix to quaternion.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> RotationMatrixToQuaternion(
    const Eigen::MatrixBase<Derived> &rot) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  Eigen::Matrix<typename Derived::Scalar, 4, 1> q;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  typename Derived::Scalar one = static_cast<typename Derived::Scalar>(1);
  typename Derived::Scalar four = static_cast<typename Derived::Scalar>(4);
  typename Derived::Scalar T = rot.trace();
  typename Derived::Scalar pivot0 = 2. * rot(0, 0) - T;
  typename Derived::Scalar pivot1 = 2. * rot(1, 1) - T;
  typename Derived::Scalar pivot2 = 2. * rot(2, 2) - T;
  typename Derived::Scalar pivot3 = T;
  if ((pivot0 >= pivot1) && (pivot0 >= pivot2) && (pivot0 >= pivot3)) {
    q(0) = sqrt((one + pivot0) / four);
    q(1) = (one / (four * q(0))) * (rot(0, 1) + rot(1, 0));
    q(2) = (one / (four * q(0))) * (rot(0, 2) + rot(2, 0));
    q(3) = (one / (four * q(0))) * (rot(1, 2) - rot(2, 1));
  } else if ((pivot1 >= pivot0) && (pivot1 >= pivot2) && (pivot1 >= pivot3)) {
    q(1) = sqrt((one + pivot1) / four);
    q(0) = (one / (four * q(1))) * (rot(0, 1) + rot(1, 0));
    q(2) = (one / (four * q(1))) * (rot(1, 2) + rot(2, 1));
    q(3) = (one / (four * q(1))) * (rot(2, 0) - rot(0, 2));
  } else if ((pivot2 > pivot0) && (pivot2 > pivot1) && (pivot2 > pivot3)) {
    q(2) = sqrt((one + pivot2) / four);
    q(0) = (one / (four * q(2))) * (rot(0, 2) + rot(2, 0));
    q(1) = (one / (four * q(2))) * (rot(1, 2) + rot(2, 1));
    q(3) = (one / (four * q(2))) * (rot(0, 1) - rot(1, 0));
  } else {
    q(3) = sqrt((one + pivot3) / four);
    q(0) = (one / (four * q(3))) * (rot(1, 2) - rot(2, 1));
    q(1) = (one / (four * q(3))) * (rot(2, 0) - rot(0, 2));
    q(2) = (one / (four * q(3))) * (rot(0, 1) - rot(1, 0));
  }
  if (q(3) < zero) {
    q = -q;
  }
  q = q / q.norm();
  return q;
}

// Conversion from rotation matrix to roll, pitch, yaw angles.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
RotationMatrixToRollPitchYaw(const Eigen::MatrixBase<Derived>& rot) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  Eigen::Matrix<typename Derived::Scalar, 3, 1> rpy;
  rpy(1, 0) = atan2(-rot(2, 0),
                    sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
  if (std::abs(cos(rpy(1, 0))) > 1.0e-12) {
    rpy(2, 0) = atan2(rot(1, 0) / cos(rpy(1, 0)), rot(0, 0) / cos(rpy(1, 0)));
    rpy(0, 0) = atan2(rot(2, 1) / cos(rpy(1, 0)), rot(2, 2) / cos(rpy(1, 0)));
  } else if (sin(rpy(1)) > static_cast<typename Derived::Scalar>(0)) {
    rpy(2, 0) = static_cast<typename Derived::Scalar>(0);
    rpy(0, 0) = atan2(rot(0, 1), rot(1, 1));
  } else {
    rpy(2, 0) = static_cast<typename Derived::Scalar>(0);
    rpy(0, 0) = atan2(-rot(0, 1), rot(1, 1));
  }
  return rpy;
}

// Conversion from roll, pitch, yaw to rotation matrix.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> RollPitchYawToRotationMatrix(
    const Eigen::MatrixBase<Derived> &roll_pitch_yaw) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 1);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_x;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0.);
  typename Derived::Scalar one = static_cast<typename Derived::Scalar>(1.);
  rotation_matrix_x << one, zero, zero, zero, cos(roll_pitch_yaw(0)), -sin(
      roll_pitch_yaw(0)), zero, sin(roll_pitch_yaw(0)), cos(roll_pitch_yaw(0));
  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_y;
  rotation_matrix_y << cos(roll_pitch_yaw(1)), zero, sin(roll_pitch_yaw(1)), zero, one, zero, -sin(
      roll_pitch_yaw(1)), 0, cos(roll_pitch_yaw(1));
  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_z;
  rotation_matrix_z << cos(roll_pitch_yaw(2)), -sin(roll_pitch_yaw(2)), zero, sin(
      roll_pitch_yaw(2)), cos(roll_pitch_yaw(2)), zero, zero, zero, one;
  return rotation_matrix_z * rotation_matrix_y * rotation_matrix_x;
}

// Jacobian of delta_theta wrt. roll and pitch
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 2> Jacobian_deltathetaTorollpitch(
    const Eigen::MatrixBase<Derived> &q) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 3, 1> rpy;
  rpy = RotationMatrixToRollPitchYaw(QuaternionToRotationMatrix(q).transpose());
  Eigen::Matrix<typename Derived::Scalar, 3, 2> jacobian;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  typename Derived::Scalar one = static_cast<typename Derived::Scalar>(1);
  jacobian << one, zero, zero, cos(rpy(0, 0)), zero, -sin(rpy(0, 0));
  return jacobian;
}

// Quaternion to angle-axis notation.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> QuaternionToAngleAxis(
    const Eigen::MatrixBase<Derived> &quaternion) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 4, 1> angle_axis;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  typename Derived::Scalar one = static_cast<typename Derived::Scalar>(1);
  angle_axis(0) = static_cast<typename Derived::Scalar>(2)
      * acos(quaternion(3));
  if (angle_axis(0) < static_cast<typename Derived::Scalar>(1e-4)) {
    angle_axis(0) = zero;
    angle_axis.template block<3, 1>(1, 0) << one, zero, zero;
  } else {
    angle_axis.template block<3, 1>(1, 0) = quaternion.template block<3, 1>(0,
                                                                            0)
        / quaternion.template block<3, 1>(0, 0).norm();
      }
  return angle_axis;
}

// Quaternion to angle-axis notation (the 3x1 vector \theta).
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> QuaternionToAngleAxisVector(
    const Eigen::MatrixBase<Derived> &quaternion) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 3, 1> angle_axis;
  typename Derived::Scalar rot_angle = static_cast<typename Derived::Scalar>(2) * acos(quaternion(3));
  if (rot_angle < static_cast<typename Derived::Scalar>(1e-6)) {
    angle_axis.setZero();
  } else {
    angle_axis = quaternion.head(3).normalized() * rot_angle;
  }
  return angle_axis;
}

// Angle-axis to quaternion.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> AngleAxisToQuaternion(
    const Eigen::MatrixBase<Derived> &angle_axis) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 4, 1> quaternion;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  quaternion(3) = cos(
      angle_axis(0) / static_cast<typename Derived::Scalar>(2.));
  quaternion.template block<3, 1>(0, 0) = sin(
      angle_axis(0) / static_cast<typename Derived::Scalar>(2.))
      * angle_axis.template block<3, 1>(1, 0)
      / angle_axis.template block<3, 1>(1, 0).norm();
  quaternion = quaternion / quaternion.norm();
  if (quaternion(3) < zero) {
    quaternion = -quaternion;
  }
  return quaternion;
}

// Angle-axis (the 3x1 vector \theta) to quaternion.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> AngleAxisVectorToQuaternion(
    const Eigen::MatrixBase<Derived> &angle_axis) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 1);
  Eigen::Matrix<typename Derived::Scalar, 4, 1> quaternion;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  typename Derived::Scalar rot_angle = angle_axis.norm();
  if (rot_angle < static_cast<typename Derived::Scalar>(1e-6)) {
    quaternion << zero, zero, zero, static_cast<typename Derived::Scalar>(1);
  } else {
    quaternion << sin(0.5*rot_angle) / rot_angle * angle_axis, cos(0.5*rot_angle);
    quaternion.normalize();
    if (quaternion(3) < zero) {
      quaternion = -quaternion;
    }
  }
  return quaternion;
}

// Skew-symmetric (cross-product) matrix.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor>
SkewSymmetricMatrix(const Eigen::MatrixBase<Derived> &x) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 1);
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor> s;
  s << zero, -x(2), x(1), x(2), zero, -x(0), -x(1), x(0), zero;
  return s;
}

// S function for IMU skewness.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
S_func(const Eigen::MatrixBase<Derived> &x) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 1);
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  typename Derived::Scalar one = static_cast<typename Derived::Scalar>(1);
  typename Derived::Scalar c0_inv = one / cos(x(0));
  typename Derived::Scalar c2_inv = one / cos(x(2));
  typename Derived::Scalar t0 = tan(x(0));
  typename Derived::Scalar t1 = tan(x(1));
  Eigen::Matrix<typename Derived::Scalar, 3, 3> S;
  S << c0_inv, zero, -t0,
	   -t1 * c0_inv * c2_inv, c2_inv / cos(x(1)), tan(x(2)) + t0 * t1 * c2_inv,
	   zero, zero, one;
  return S;
}

// Compute the product q1 * q2.
template<typename Derived, typename DerivedOther>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> QuaternionMultiplication(
    const Eigen::MatrixBase<Derived> &q1,
    const Eigen::MatrixBase<DerivedOther> &q2) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedOther>, 4,
                                           1);
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> L;
  L.template block<3, 3>(0, 0) = q1(3)
      * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
      - SkewSymmetricMatrix(q1.template block<3, 1>(0, 0));
  L.template block<1, 3>(3, 0) = -q1.template block<3, 1>(0, 0).transpose();
  L.template block<4, 1>(0, 3) = q1;
  Eigen::Matrix<typename Derived::Scalar, 4, 1> result = L * q2;
  result = result / result.norm();
  if (result(3) < zero) {
    result = -result;
  }
  return result;
}

// Compute the product  [0.5 * delta_theta ; 1] * q1
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> CorrectionToQuaternion(
    const Eigen::MatrixBase<Derived> &delta_theta) {
  Eigen::Matrix<typename Derived::Scalar, 4, 1> delta_q;
  typename Derived::Scalar point_five;
  typename Derived::Scalar one_point_zero;
  point_five = 0.5;
  one_point_zero = 1.0;
  delta_q.block(0, 0, 3, 1) = point_five * delta_theta;
  delta_q(3, 0) = one_point_zero;
  delta_q = (delta_q / delta_q.norm()).eval();
  return delta_q;
}

// Compute the product  [0.5 * delta_theta ; 1] * q1
template<typename DerivedOther, typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> UpdateQuaternion(
    const Eigen::MatrixBase<DerivedOther> &delta_theta,
    const Eigen::MatrixBase<Derived> &q1) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedOther>, 3,
                                           1);
  Eigen::Matrix<typename Derived::Scalar, 4, 1> correction_quaternion;
  correction_quaternion = CorrectionToQuaternion(delta_theta);
  return QuaternionMultiplication(correction_quaternion, q1);
}

// Compute left multiplication matrix.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuaternionMultiplicationMatrix(
    const Eigen::MatrixBase<Derived> &q) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> L;
  L.template block<3, 3>(0, 0) = q(3)
      * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
      - SkewSymmetricMatrix(q.template block<3, 1>(0, 0));
  L.template block<3, 1>(0, 3) = q.template block<3, 1>(0, 0);
  L.template block<1, 3>(3, 0) = -q.template block<3, 1>(0, 0).transpose();
  L(3, 3) = q(3);
  return L;
}

// Compute right multiplication matrix.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuaternionMultiplicationMatrix(
    const Eigen::MatrixBase<Derived> &q) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> R;
  R.template block<3, 3>(0, 0) = q(3)
      * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
      + SkewSymmetricMatrix(q.template block<3, 1>(0, 0));
  R.template block<3, 1>(0, 3) = q.template block<3, 1>(0, 0);
  R.template block<1, 3>(3, 0) = -q.template block<3, 1>(0, 0).transpose();
  R(3, 3) = q(3);
  return R;
}

// Magnitude of Earth's gravitational field at specific height [m] and
// latitude [rad]. This is from wikipedia, we can
inline double MagnitudeOfGravity(const double height, const double latitude) {
  double sin_squared_latitude = sin(latitude) * sin(latitude);
  double sin_squared_twice_latitude = sin(2 * latitude) * sin(2 * latitude);
  return 9.780327
      * ((1 + 0.0053024 * sin_squared_latitude
          - 0.0000058 * sin_squared_twice_latitude) - 3.155 * 1e-7 * height);
}

inline Eigen::Vector3d QuaternionRotateVector(const Eigen::Vector4d &A_q_B,
                                              const Eigen::Vector3d &B_vector) {
  return QuaternionToRotationMatrix(A_q_B) * B_vector;
}

inline Eigen::Matrix<double, 4, 1> RandomQuaternion() {
  Eigen::Matrix<double, 4, 1> q;
  q.setRandom();
  q.array() -= 0.5;
  q.normalize();
  if (q[3] < 0)
    q = -q;
  return q;
}
inline Eigen::Matrix<double, 4, 1> IdentityQuaternion() {
  Eigen::Matrix<double, 4, 1> q = Eigen::Matrix<double, 4, 1>::Zero();
  q[3] = 1.0;
  return q;
}

inline Eigen::Vector3d InterpolatePosition(Eigen::Vector3d& p1, Eigen::Vector3d& p2, double lamda){
  Eigen::Vector3d p = (1-lamda) * p1 + lamda * p2;
  return p;
}
inline Eigen::Matrix3d InterpolateRotation(Eigen::Matrix3d& C1, Eigen::Matrix3d& C2, double lamda){
  Eigen::Matrix3d C = (1-lamda) * C1 + lamda * C2;
  return C;
}

template<typename Scalar>
inline Scalar Sign(Scalar value) {
  if (value > 0)
    return 1.;
  if (value < 0)
    return -1.;
  return 0.;
}

template<typename scalar>
inline Eigen::Matrix<scalar, 3, 3, Eigen::RowMajor> RotationX(scalar theta) {
  scalar ct = cos(theta);
  scalar st = sin(theta);
  Eigen::Matrix<scalar, 3, 3, Eigen::RowMajor> rotation_matrix;
  rotation_matrix << 1, 0, 0, 0, ct, -st, 0, st, ct;
  return rotation_matrix;
}

template<typename scalar>
inline Eigen::Matrix<scalar, 3, 3, Eigen::RowMajor> RotationY(scalar theta) {
  scalar ct = cos(theta);
  scalar st = sin(theta);
  Eigen::Matrix<scalar, 3, 3, Eigen::RowMajor> rotation_matrix;
  rotation_matrix << ct, 0, st, 0, 1, 0, -st, 0, ct;
  return rotation_matrix;
}

template<typename scalar>
inline Eigen::Matrix<scalar, 3, 3, Eigen::RowMajor> RotationZ(scalar theta) {
  scalar ct = cos(theta);
  scalar st = sin(theta);
  Eigen::Matrix<scalar, 3, 3, Eigen::RowMajor> rotation_matrix;
  rotation_matrix << ct, -st, 0, st, ct, 0, 0, 0, 1;
  return rotation_matrix;
}

// Chi-square thresholds for .95 percentile (1 dof, 2 dof, etc.)
const float CHI_THRESH[500] = { 3.841459, 5.991465, 7.814728, 9.487729,
    11.070498, 12.591587, 14.067140, 15.507313, 16.918978, 18.307038, 19.675138,
    21.026070, 22.362032, 23.684791, 24.995790, 26.296228, 27.587112, 28.869299,
    30.143527, 31.410433, 32.670573, 33.924438, 35.172462, 36.415029, 37.652484,
    38.885139, 40.113272, 41.337138, 42.556968, 43.772972, 44.985343, 46.194260,
    47.399884, 48.602367, 49.801850, 50.998460, 52.192320, 53.383541, 54.572228,
    55.758479, 56.942387, 58.124038, 59.303512, 60.480887, 61.656233, 62.829620,
    64.001112, 65.170769, 66.338649, 67.504807, 68.669294, 69.832160, 70.993453,
    72.153216, 73.311493, 74.468324, 75.623748, 76.777803, 77.930524, 79.081944,
    80.232098, 81.381015, 82.528727, 83.675261, 84.820645, 85.964907, 87.108072,
    88.250164, 89.391208, 90.531225, 91.670239, 92.808270, 93.945340, 95.081467,
    96.216671, 97.350970, 98.484383, 99.616927, 100.748619, 101.879474,
    103.009509, 104.138738, 105.267177, 106.394840, 107.521741, 108.647893,
    109.773309, 110.898003, 112.021986, 113.145270, 114.267868, 115.389790,
    116.511047, 117.631651, 118.751612, 119.870939, 120.989644, 122.107735,
    123.225221, 124.342113, 125.458419, 126.574148, 127.689308, 128.803908,
    129.917955, 131.031458, 132.144425, 133.256862, 134.368777, 135.480178,
    136.591071, 137.701464, 138.811363, 139.920774, 141.029704, 142.138160,
    143.246147, 144.353672, 145.460740, 146.567358, 147.673530, 148.779262,
    149.884561, 150.989430, 152.093876, 153.197903, 154.301516, 155.404721,
    156.507522, 157.609923, 158.711930, 159.813547, 160.914778, 162.015628,
    163.116101, 164.216201, 165.315932, 166.415299, 167.514305, 168.612954,
    169.711251, 170.809198, 171.906799, 173.004059, 174.100981, 175.197567,
    176.293823, 177.389750, 178.485353, 179.580634, 180.675597, 181.770246,
    182.864582, 183.958610, 185.052332, 186.145751, 187.238870, 188.331692,
    189.424220, 190.516457, 191.608404, 192.700066, 193.791445, 194.882542,
    195.973362, 197.063906, 198.154177, 199.244177, 200.333909, 201.423375,
    202.512577, 203.601519, 204.690201, 205.778627, 206.866798, 207.954717,
    209.042386, 210.129807, 211.216982, 212.303913, 213.390602, 214.477052,
    215.563263, 216.649239, 217.734981, 218.820491, 219.905770, 220.990822,
    222.075646, 223.160247, 224.244624, 225.328780, 226.412716, 227.496435,
    228.579938, 229.663226, 230.746302, 231.829167, 232.911822, 233.994269,
    235.076510, 236.158546, 237.240378, 238.322009, 239.403439, 240.484671,
    241.565705, 242.646544, 243.727187, 244.807638, 245.887897, 246.967965,
    248.047844, 249.127536, 250.207041, 251.286361, 252.365498, 253.444451,
    254.523224, 255.601816, 256.680230, 257.758465, 258.836525, 259.914409,
    260.992120, 262.069657, 263.147023, 264.224218, 265.301243, 266.378101,
    267.454791, 268.531314, 269.607673, 270.683868, 271.759900, 272.835769,
    273.911478, 274.987027, 276.062417, 277.137650, 278.212725, 279.287644,
    280.362409, 281.437019, 282.511477, 283.585782, 284.659936, 285.733940,
    286.807794, 287.881501, 288.955059, 290.028471, 291.101737, 292.174858,
    293.247835, 294.320669, 295.393360, 296.465910, 297.538319, 298.610588,
    299.682719, 300.754710, 301.826565, 302.898282, 303.969864, 305.041310,
    306.112622, 307.183800, 308.254846, 309.325759, 310.396541, 311.467192,
    312.537713, 313.608105, 314.678368, 315.748503, 316.818512, 317.888393,
    318.958149, 320.027780, 321.097286, 322.166669, 323.235928, 324.305065,
    325.374080, 326.442974, 327.511748, 328.580401, 329.648936, 330.717351,
    331.785649, 332.853829, 333.921892, 334.989839, 336.057670, 337.125386,
    338.192988, 339.260476, 340.327850, 341.395112, 342.462262, 343.529300,
    344.596226, 345.663043, 346.729749, 347.796346, 348.862834, 349.929214,
    350.995485, 352.061650, 353.127708, 354.193659, 355.259504, 356.325245,
    357.390880, 358.456412, 359.521839, 360.587163, 361.652385, 362.717504,
    363.782521, 364.847437, 365.912253, 366.976967, 368.041582, 369.106097,
    370.170513, 371.234831, 372.299051, 373.363173, 374.427197, 375.491125,
    376.554957, 377.618692, 378.682332, 379.745878, 380.809328, 381.872684,
    382.935947, 383.999116, 385.062192, 386.125175, 387.188067, 388.250867,
    389.313575, 390.376192, 391.438719, 392.501156, 393.563503, 394.625760,
    395.687929, 396.750009, 397.812000, 398.873904, 399.935720, 400.997450,
    402.059092, 403.120648, 404.182118, 405.243502, 406.304801, 407.366015,
    408.427145, 409.488190, 410.549151, 411.610029, 412.670823, 413.731535,
    414.792164, 415.852711, 416.913176, 417.973559, 419.033862, 420.094083,
    421.154224, 422.214284, 423.274265, 424.334166, 425.393988, 426.453731,
    427.513395, 428.572980, 429.632488, 430.691918, 431.751271, 432.810546,
    433.869745, 434.928867, 435.987913, 437.046882, 438.105777, 439.164596,
    440.223339, 441.282008, 442.340603, 443.399123, 444.457570, 445.515942,
    446.574242, 447.632468, 448.690621, 449.748702, 450.806711, 451.864647,
    452.922512, 453.980305, 455.038027, 456.095679, 457.153259, 458.210769,
    459.268209, 460.325579, 461.382879, 462.440110, 463.497272, 464.554365,
    465.611389, 466.668344, 467.725232, 468.782052, 469.838804, 470.895488,
    471.952105, 473.008656, 474.065139, 475.121556, 476.177907, 477.234192,
    478.290411, 479.346565, 480.402653, 481.458676, 482.514634, 483.570528,
    484.626357, 485.682122, 486.737823, 487.793460, 488.849033, 489.904544,
    490.959991, 492.015375, 493.070697, 494.125956, 495.181153, 496.236287,
    497.291360, 498.346372, 499.401322, 500.456210, 501.511038, 502.565805,
    503.620511, 504.675157, 505.729742, 506.784268, 507.838733, 508.893140,
    509.947486, 511.001774, 512.056002, 513.110172, 514.164283, 515.218335,
    516.272329, 517.326265, 518.380143, 519.433964, 520.487727, 521.541432,
    522.595081, 523.648672, 524.702207, 525.755685, 526.809107, 527.862472,
    528.915781, 529.969035, 531.022232, 532.075374, 533.128461, 534.181492,
    535.234469, 536.287390, 537.340257, 538.393069, 539.445827, 540.498531,
    541.551181, 542.603777, 543.656319, 544.708807, 545.761243, 546.813625,
    547.865954, 548.918230, 549.970453, 551.022624, 552.074743, 553.126809 };  // NOLINT

} /** End of namespace: Geometry */
#endif  /** End of file: GEOMETRY_H_ */
