#ifndef FIVE_POINT_SOLVER_H_
#define FIVE_POINT_SOLVER_H_

#include <Eigen/Core> 
#include <Eigen/Dense>
#include <vector>
#include <geometry/geometry.h>
#include "ransac-solver/ransac-solver.h"	/// To use: RansacSolver
/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
/**	@class FivePointSolver
 * 	@brief The FivePointSolver class.
 *	This class implements the five point algorithm for Ransac solver.
 */
class FivePointSolver : public RansacSolver {
 protected:
  // Data Members
  Eigen::Matrix3d c2_R_c1;	// The quaternion representing the orientation of frame1 w.r.t. frame2
  Eigen::Vector3d c2_t_c1;	// The unit translation vector of frame1 w.r.t. frame2
  Eigen::MatrixXd measurements_frame1;	/// The homogeneuos coordinates of the features in the frame1
  Eigen::MatrixXd measurements_frame2;	/// The homogeneuos coordinates of the features in the frame2

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor and Destructor
  FivePointSolver();
  FivePointSolver(double _error_tolerance, double _rejection_percent);
  virtual ~FivePointSolver();

  /// Setters and Getters
  void setMeasurements(Eigen::MatrixXd& _measurements_frame1,
                       Eigen::MatrixXd& _measurements_frame2) {
    measurements_frame1 = _measurements_frame1;
    measurements_frame2 = _measurements_frame2;
    data_size = _measurements_frame1.cols();
  }
  inline Eigen::Vector4d getRotationQuaternion_2Q1() {
    return Geometry::RotationMatrixToQuaternion(c2_R_c1);
  }
  inline Eigen::Matrix3d getRotationMatrix_2R1() {
    return c2_R_c1;
  }
  inline Eigen::Vector3d getTranslation_2t1() {
    return c2_t_c1;
  }

  /// Operation Functions
  /// @fn SolveMinimal, Estimate the unit translation vector based on the minimal problem,
  /// given the indices of the points used in the estimation.
  void SolveMinimal(const std::vector<int>& indexes);
  /// @fn GetInliers, Classify the datapoint to inliers and outliers based on the epipolar constraint.
  void GetInliers(std::vector<int> &inlier_index,
                  std::vector<int> &outlier_index);

};
/** End of class: FivePointSolver */
} /** End of namespace: Ransac */
#endif	/** End of file: FIVE_POINT_SOLVER_H_*/
