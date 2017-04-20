#ifndef RANSAC_SOLVER_H_
#define RANSAC_SOLVER_H_

#include <iostream>		/// To use: cout, ostream
#include <vector>		/// To use: vector
/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
/**	@class RansacSolver
 * 	@brief The RansacSolver class.
 *	The main Ransac solver class.
 */
class RansacSolver {
 protected:
  /// Protected data members.
  /// User Control Parameters
  double error_tolerance;		/// Error tolerance threshold for accepting inliers.
  double size_rejection_percent;	/// Lower bound percentage of outlier features to reject the hypothesis.
  /// Solver Specific Parameters
  unsigned int data_size;  /// Number of elements of the data set (i.e. set when setting the input).
  unsigned int min_set_size;	/// Minimum size of set used to generate hypothesis  .
  unsigned int solution_type;		///
 public:
  /// Constructors and Destructor
  RansacSolver() {
    size_rejection_percent = 0.2;
    error_tolerance = 0;
    min_set_size = 0;
    solution_type = 0;
    data_size = 0;
  }
  RansacSolver(double _error_tolerance, double _rejection_percent) {
    error_tolerance = _error_tolerance;
    size_rejection_percent = _rejection_percent;
    min_set_size = 0;
    solution_type = 0;
    data_size = 0;
  }
  virtual ~RansacSolver() {
  }
  /// Setters and Getters
  /// Set the size rejection percent
  inline void setSizeRejectionPercent(double percent) {
    size_rejection_percent = percent;
  }
  inline double getSizeRejectionPercent() {
    return size_rejection_percent;
  }
  inline void setErrorTolerance(double _error_tolerance) {
    error_tolerance = _error_tolerance;
  }
  inline double getErrorTolerance() {
    return error_tolerance;
  }
  inline unsigned int getSolutionType() {
    return solution_type;
  }
  inline unsigned int getMinimumSetSize() {
    return min_set_size;
  }
  inline unsigned int GetSetSize() {
    return data_size;
  }

  /// Operation Functions
  /// @fn GenerateHypothesis, Run the solver based on the number of points,
  ///	to estimate the hypothesis for Ransac.
  inline void GenerateHypothesis(const std::vector<int>& indexes_of_points) {
    if (indexes_of_points.size() < min_set_size) {
      ///perror("No enough data points for Ransac solver");
      return;
    }
    if (indexes_of_points.size() == min_set_size) {
      SolveMinimal(indexes_of_points);
    } else {
      SolveWithInliers(indexes_of_points);
    }
  }
  /// @fn SolveMinimal, Estimate the unit translation vector based on the minimal problem,
  /// given the indices of the points used in the estimation.
  virtual void SolveMinimal(const std::vector<int>& indexes)=0;
  /// @fn SolveWithInliers, Estimate the unit translation vector based on the epipolar constraint,
  /// given the indices of the points used in the estimation (i.e., estimated inliers).
  virtual void SolveWithInliers(const std::vector<int>& indexes_of_inliers) {
    (void)indexes_of_inliers;
  }
  /// @fn GetInliers, Classify the datapoint to inliers and outliers based on the epipolar constraint.
  virtual void GetInliers(std::vector<int> &inlier_index,
                          std::vector<int> &outlier_index)=0;
};
/** End of class: RansacSolver*/
} /** End of namespace: Ransac */
#endif	/** End of File: RANSAC_SOLVER_H_*/
