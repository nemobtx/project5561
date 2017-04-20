#ifndef TWO_POINT_SOLVER_H_
#define TWO_POINT_SOLVER_H_

#include <float.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ransac-solver/ransac-solver.h"	/// To use: RansacSolver
#include <geometry/geometry.h>
/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
/**	@class TwoPointSolver
 * 	@brief The TwoPointSolver class.
 *	The two point solver for the Ransac algorithm.
 */
class TwoPointSolver : public RansacSolver {
 private:
  // Input
  double sufficient_angle;
  int error_type;
  Eigen::Matrix3d c2_R_c1;				/// The relative orientation
  Eigen::MatrixXd measurements_frame1;	/// The homogeneuos coordinates of the features in the frame1
  Eigen::MatrixXd measurements_frame2;	/// The homogeneuos coordinates of the features in the frame2
  // Output
  Eigen::Vector3d c2_t_c1;	// The estimated unit translation vector
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TwoPointSolver();
  TwoPointSolver(double _error_tolerance,
                 double _rejection_percent, double _sufficient_angle,
                 int error_type);
  virtual ~TwoPointSolver() {
  }
  /// Setters and Getters
  inline void setSufficientAngle(double _sufficient_angle) {
    sufficient_angle = _sufficient_angle;
  }
  inline double getSufficientAngle() {
    return sufficient_angle;
  }
  inline void setErrorType(int _error_type) {
    error_type = _error_type;
  }
  inline int getErrorType() {
    return error_type;
  }
  inline void setRelativeOrientation2Q1(const Eigen::Vector4d& _c2_q_c1) {
    c2_R_c1 = Geometry::QuaternionToRotationMatrix(_c2_q_c1);
  }
  inline void setRelativeOrientation2R1(const Eigen::Matrix3d& _c2_R_c1) {
    c2_R_c1 = _c2_R_c1;
  }
  void setMeasurements(Eigen::MatrixXd& _measurements_frame1,
                       Eigen::MatrixXd& _measurements_frame2) {
    measurements_frame1 = _measurements_frame1;
    measurements_frame2 = _measurements_frame2;
    data_size = _measurements_frame1.cols();
  }
  inline Eigen::Vector3d GetEstimatedTranslation() {
    return c2_t_c1;
  }
  /// Operation Functions
  /// @fn SolveMinimal, Estimate the unit translation vector based on the minimal problem,
  /// given the indices of the points used in the estimation.
  void SolveMinimal(const std::vector<int>& indexes);
  /// @fn SolveWithInliers, Estimate the unit translation vector based on the epipolar constraint,
  /// given the indices of the points used in the estimation (i.e., estimated inliers).
  void SolveWithInliers(const std::vector<int>& indexes_of_inliers);
  /// @fn GetInliers, Classify the datapoint to inliers and outliers based on the epipolar constraint.
  void GetInliers(std::vector<int> &inlier_index,
                  std::vector<int> &outlier_index);
  double GetMaxBearingAngle(const std::vector<int>& indexes_of_inliers);

};
/** End of class: TwoPointSolver */
} /** End of namespace: Ransac */
#endif	/** End of file: TWO_POINT_SOLVER_H_ */
