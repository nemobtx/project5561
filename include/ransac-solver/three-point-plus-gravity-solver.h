#ifndef THREE_POINT_PLUS_GRAVITY_SOLVER_H_
#define THREE_POINT_PLUS_GRAVITY_SOLVER_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <geometry/geometry.h>
#include "ransac-solver/ransac-solver.h"	/// To use: RansacSolver
/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
/**	@class ThreePointPlusGravitySolver
 * 	@brief The ThreePointPlusGravitySolver class.
 *	This class implements the five point algorithm for Ransac solver.
 */
class ThreePointPlusGravitySolver : public RansacSolver {
 protected:
  // Data Members
  Eigen::Matrix3d c2_R_c1_;	// The orientation of frame1 w.r.t. frame2
  Eigen::Vector3d c2_t_c1_;	// The unit translation vector of frame1 w.r.t. frame2
  Eigen::MatrixXd measurements_frame1;	/// The homogeneuos coordinates of the features in the frame1
  Eigen::MatrixXd measurements_frame2;	/// The homogeneuos coordinates of the features in the frame2
  Eigen::Vector3d frame1_g;
  Eigen::Vector3d frame2_g;
  bool pure_rotation_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor and Destructor
  ThreePointPlusGravitySolver();
  ThreePointPlusGravitySolver(double _error_tolerance, double _rejection_percent);
  virtual ~ThreePointPlusGravitySolver();
  void set_pure_rotation(bool pure_rotation) {
    pure_rotation_ = pure_rotation;
  }
  /// Setters and Getters
  void SetMeasurements(const Eigen::MatrixXd& _measurements_frame1,
                       const Eigen::MatrixXd& _measurements_frame2,
                       const Eigen::Vector3d& _frame1_g,
                       const Eigen::Vector3d& _frame2_g) {
    measurements_frame1 = _measurements_frame1;
    measurements_frame2 = _measurements_frame2;
    frame1_g = _frame1_g;
    frame2_g = _frame2_g;
    data_size = _measurements_frame1.cols();
  }
  inline Eigen::Matrix3d c2_R_c1() {
    return c2_R_c1_;
  }
  inline Eigen::Vector3d c2_t_c1() {
    return c2_t_c1_;
  }
  /// Operation Functions
  /// @fn SolveMinimal, Estimate the unit translation vector based on the minimal problem,
  /// given the indices of the points used in the estimation.
  void SolveMinimal(const std::vector<int>& indexes);
  /// @fn GetInliers, Classify the datapoint to inliers and outliers based on the epipolar constraint.
  void GetInliers(std::vector<int> &inlier_index,
                  std::vector<int> &outlier_index);
};
/** End of class: ThreePointPlusGravitySolver */
} /** End of namespace: Ransac */
#endif	/** End of file: */
