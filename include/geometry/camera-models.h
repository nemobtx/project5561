#ifndef CAMERA_MODELS_H_
#define CAMERA_MODELS_H_

#include <Eigen/Core>

/** @namespace Geometry
 * The Geometry namespace.
 */
namespace Geometry {

/*
inline void normalizeUndistortScalars(float x, float y, float &u, float &v,
		Eigen::Matrix<float, 1, 2> fc,
		Eigen::Matrix<float, 1, 5>  kc,
		Eigen::Matrix<float, 1, 2>  cc,
		int iterations) {



	x = (x - cc(0, 0)) / fc(0, 0);
	y = (y - cc(0, 1)) / fc(0, 1);
	float k1 = kc(0,0);
	float k2 = kc(0,1);
	float k3 = kc(0,4);
	float p1 = kc(0,2);
	float p2 = kc(0,3);

	float k_radial = 0;
	float r_2 = 0;
	float xd0;xd0 = x;
	float xd1;xd1 = y;
	u = x;
	v = y;
	for (int i = 0; i < iterations; i++) {
		float p00, p11, p01;
		p00 = u * u;// * point(0);
		p11 = v * v;//point(1) * point(1);
		p01 = u * v;//point(0) * point(1);
		r_2 = p00 + p11;
		k_radial = 1 / (1 + (k1 * r_2) + (k2 * r_2 * r_2) + (k3 * r_2 * r_2 * r_2));
		u = (xd0 - (2 * p1 * p01 + p2 * (r_2 + 2 * p00))) * k_radial;
		v = (xd1 - (p1 * (r_2 + 2 * p11) + 2 * p2 * p01)) * k_radial;
	}
}
*/

inline int normalizeUndistortScalarsTango(float x, float y, float &u, float &v,
                                   Eigen::Matrix<float, 1, 2> fc,
                                   Eigen::Matrix<float, 1, 5>  kc,
                                   Eigen::Matrix<float, 1, 2>  cc,
                                   int iterations) {
  (void)iterations;
//  *** MATLAB Code ***
//  function point_out = undistort_tango(point_in,cx, cy, fx, fy,w)
//
//  point_normalize(1) = (point_in(1)-cx)/fx;
//  point_normalize(2) = (point_in(2)-cy)/fy;
//
//  mul2tanwby2 = tan(w/2) * 2;
//  r_d = norm(point_normalize);
//
//  if (abs(r_d*w) <= pi*89/180)
//      r_u = tan(r_d*w)/(r_d*mul2tanwby2);
//      point_out = r_u * point_normalize;
//  else
//      point_out = [-1; -1];
//  end
// ***

  x = (x - cc(0, 0)) / fc(0, 0);
  y = (y - cc(0, 1)) / fc(0, 1);
  u = x;
  v = y;
  float omega = kc(0,0);
  float r_d = sqrt(x * x + y * y);
  float mul2tanwby2 = tan(omega / 2) * 2;
  if (fabs(r_d * mul2tanwby2) < 0.0001)
    return 1;
  if (fabs(r_d * omega) <= 3.1415 * 89.0 / 180.0) {
    float r_u = tan(r_d * omega) / (r_d * mul2tanwby2);
    u = r_u * x;
    v = r_u * y;
    return 0;
  } else {
    u = -1;
    v = -1;
    return 0;
  }
};

Eigen::Vector2d distortFisheye(const Eigen::Vector2d& point,
                               const Eigen::Vector2d& fc,
                               const Eigen::Matrix<double, 5, 1>& kc,
                               const Eigen::Matrix<double, 2, 1>& cc);

Eigen::Vector2d distortTango(const Eigen::Vector2d& point,
                              const Eigen::Vector2d& fc,
                              const Eigen::Matrix<double, 5, 1>& kc,
                              const Eigen::Vector2d& cc);

Eigen::Vector2d un_normalize(const Eigen::Vector3d& norm_point,
                             const Eigen::Vector2d& fc,
                             const Eigen::Matrix<double, 5, 1>& kc,
                             const Eigen::Matrix<double, 2, 1>& cc);

Eigen::Vector3d normalize_me(Eigen::Vector2d &pixel_point_, Eigen::Vector2d &fc,
                             Eigen::Matrix<double, 5, 1> &kc,
                             Eigen::Matrix<double, 2, 1> &cc);
int normalizeUndistortScalars(float x, float y, float &u, float &v,
                               const Eigen::Vector2d& fc,
                               const Eigen::VectorXd& kc,
                               const Eigen::MatrixXd& cc, int iterations);

int normalizeUndistortScalarsTango(float x, float y, float &u, float &v,
                                    const Eigen::Vector2d& fc,
                                    const Eigen::VectorXd& kc,
                                    const Eigen::MatrixXd& cc, int iterations);

int normalizeUndistortFisheye(float x, float y, double &u, double &v,
                              const Eigen::Vector2d& fc,
                              const Eigen::VectorXd& kc,
                              const Eigen::MatrixXd& cc,
                              int iterations);

int normalizeUndistortScalars(float x, float y, double &u, double &v,
                                    const Eigen::Vector2d& fc,
                                    const Eigen::VectorXd& kc,
                                    const Eigen::MatrixXd& cc, int iterations);
int normalizeUndistortScalarsTango(float x, float y, double &u, double &v,
                                    const Eigen::Vector2d& fc,
                                    const Eigen::VectorXd& kc,
                                    const Eigen::MatrixXd& cc, int iterations);

template<typename Derived1, typename Derived2, typename Derived3,
         typename Derived4, typename Derived5>
inline int normalizeUndistortTango(const Eigen::MatrixBase<Derived1> & C_px_f,
                                   const Eigen::MatrixBase<Derived2> & fc,
                                   const Eigen::MatrixBase<Derived3> & kc,
                                   const Eigen::MatrixBase<Derived4> & cc,
                                   const Eigen::MatrixBase<Derived5> & _C_u_f) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived1>, 2, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived2>, 2, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived4>, 2, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived5>, 2, 1);
  Eigen::MatrixBase<Derived5> & C_u_f =
      const_cast<Eigen::MatrixBase<Derived5> &>(_C_u_f);
  double x = (C_px_f(0) - cc(0, 0)) / fc(0, 0);
  double y = (C_px_f(1) - cc(1, 0)) / fc(1, 0);
  C_u_f(0) = x;
  C_u_f(1) = y;
  double omega = kc(0);
  double r_d = std::sqrt(x * x + y * y);
  double mul2tanwby2 = std::tan(omega / 2) * 2;
  if (std::abs(r_d * mul2tanwby2) < 1e-6) {
    return 1;
  }
  if (std::abs(r_d * omega) <= M_PI * 89.0 / 180.0) {
    double r_u = std::tan(r_d * omega) / (r_d * mul2tanwby2);
    C_u_f(0) = r_u * x;
    C_u_f(1) = r_u * y;
    return 0;
  } else {
    C_u_f(0) = -1;
    C_u_f(1) = -1;
    return 1;
  }
}

/// Normalizing and undistoring like in Camera Calibration Toolbox
template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
inline Eigen::Vector3d normalizeUndistort(const Eigen::MatrixBase<Derived1> & point,
                                          const Eigen::MatrixBase<Derived2> & fc,
                                          const Eigen::MatrixBase<Derived3> & kc,
                                          const Eigen::MatrixBase<Derived4> & cc) {
//  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived2>, 2, 1);
//  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived3>, 5, 1);
//  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived4>, 2, 1);

  Eigen::Vector3d normalized;
  normalized(0) = (point(0) - cc(0, 0)) / fc(0, 0);
  normalized(1) = (point(1) - cc(1, 0)) / fc(1, 0);
  double k1 = kc(0);
  double k2 = kc(1);
  double k3 = kc(4);
  double p1 = kc(2);
  double p2 = kc(3);

  double k_radial = 0;
  double r_2 = 0;
  Eigen::Vector2d xd;
  xd = normalized.block(0, 0, 2, 1);
  Eigen::Vector2d delta_x;
  for (int i = 0; i < 20; i++) {
    double p00, p11, p01;
    p00 = normalized(0) * normalized(0);
    p11 = normalized(1) * normalized(1);
    p01 = normalized(0) * normalized(1);
    r_2 = p00 + p11;
    k_radial = 1 + (k1 * r_2) + (k2 * r_2 * r_2) + (k3 * r_2 * r_2 * r_2);
    delta_x << 2 * p1 * p01 + p2 * (r_2 + 2 * p00), p1 * (r_2 + 2 * p11)
        + 2 * p2 * p01;
    normalized.block(0, 0, 2, 1) = (xd - delta_x) / k_radial;
  }
  normalized(2, 0) = 1.0;
  return normalized;
}
/// @fn normalizeUndistortFisheye, Normalizing and undistort fish eye lenses
Eigen::Vector3d normalizeUndistortFisheye(Eigen::Vector3d point,
                                          Eigen::Vector2d fc,
                                          Eigen::VectorXd kc,
                                          Eigen::MatrixXd cc);

/// @fn normalizeUndistortGeneric,
inline Eigen::Vector3d normalizeUndistortGeneric(Eigen::Vector3d point,
                                                 Eigen::Vector2d fc,
                                                 Eigen::VectorXd kc,
                                                 Eigen::Vector2d cc,
                                                 bool fisheye) {
  if (fisheye) {
    return normalizeUndistortFisheye(point, fc, kc, cc);
  } else {
    return normalizeUndistort(point, fc, kc, cc);
  }
}

/// @fn denormalizeDistort,
inline Eigen::Vector3d denormalizeDistort(Eigen::Vector3d point,
                                          Eigen::Vector2d fc,
                                          Eigen::MatrixXd kc,
                                          Eigen::MatrixXd alpha_c,
                                          Eigen::MatrixXd cc) {
  (void)alpha_c;
  double r2 = point(0) * point(0) + point(1) * point(1);
  // Distort the coordinates first
  point = (1 + kc(0) * r2 + kc(1) * r2 * r2) * point;
  // Now move back into the image frame
  point(0) = fc(0) * point(0) + cc(0);
  point(1) = fc(1) * point(1) + cc(1);

  return point;
}
/*pinhole projection model*/
inline Eigen::Vector2d pinholePixel(const Eigen::Vector3d &pwrtCamera) {
  Eigen::Vector2d pixel;
  pixel(0) = pwrtCamera(0) / pwrtCamera(2);
  pixel(1) = pwrtCamera(1) / pwrtCamera(2);
  return pixel;
}

/*pinhole projection model*/
inline Eigen::Vector3d pinholeHomogeneous(const Eigen::Vector3d &pwrtCamera) {
  Eigen::Vector3d pixel;
  pixel(0) = pwrtCamera(0) / pwrtCamera(2);
  pixel(1) = pwrtCamera(1) / pwrtCamera(2);
  pixel(2) = 1;
  return pixel;
}

} /** End of namespace: Geometry */
#endif /** End of file: CAMERA_MODELS_H_ */
