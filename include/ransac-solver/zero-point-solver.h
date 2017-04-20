#ifndef ZERO_POINT_SOLVER_H_
#define ZERO_POINT_SOLVER_H_

#include <Eigen/Core> 
#include "ransac-solver/ransac-solver.h"  /// To use: RansacSolver
#include <geometry/geometry.h>
/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
class ZeroPointSolver : public RansacSolver {
 private:
  Eigen::Matrix3d c2_R_c1;
  Eigen::MatrixXd measurements_frame1;  /// The homogeneuos coordinates of the features in the frame1
  Eigen::MatrixXd measurements_frame2;  /// The homogeneuos coordinates of the features in the frame2
  Eigen::Vector3d c2_t_c1;
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ZeroPointSolver() {
    c2_R_c1.setIdentity();
    error_tolerance = 0;
    min_set_size = 0;
    solution_type = 0;
    c2_t_c1.setZero();
  }
  ZeroPointSolver(double _error_tolerance, double _rejection_percent = 1)
      : RansacSolver(_error_tolerance, _rejection_percent) {
    c2_R_c1.setIdentity();
    min_set_size = 0;
    solution_type = 0;
    c2_t_c1.setZero();
  }
  virtual ~ZeroPointSolver() {
  }
  void setMeasurements(Eigen::MatrixXd& _measurements_frame1,
                       Eigen::MatrixXd& _measurements_frame2) {
    measurements_frame1 = _measurements_frame1;
    measurements_frame2 = _measurements_frame2;
    data_size = _measurements_frame1.cols();
  }
  inline void setRelativeOrientation2Q1(const Eigen::Vector4d& _c2_q_c1) {
    c2_R_c1 = Geometry::QuaternionToRotationMatrix(_c2_q_c1);
  }
  inline void setRelativeOrientation2R1(const Eigen::Matrix3d& _c2_R_c1) {
    c2_R_c1 = _c2_R_c1;
  }
  void SolveMinimal(const std::vector<int>& indexes) {
    (void)indexes;
  }
  void GetInliers(std::vector<int> &inlier_index,
                  std::vector<int> &outlier_index);
};
/** End of class: ZeroPointSolver */
} /** End of namespace: Ransac */
#endif  /** End of file: ZERO_POINT_SOLVER_H_ */
