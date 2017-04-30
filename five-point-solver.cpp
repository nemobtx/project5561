#include "ransac-solver/five-point-solver.h"	/// To use: FivePointSolver
#include <geometry/geometry.h>
#include <geometry/vision.h>
#include <geometry/error.h>
#include <Eigen/Eigenvalues>

#include <math.h>		// To use: fabs, isnan
#include <float.h>		// To use: DBL_MAX 
#include <algorithm>

/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
/// Constructor
FivePointSolver::FivePointSolver() {
  min_set_size = 5;
  solution_type = 5;
  c2_R_c1.setIdentity();  // The quaternion representing the orientation of frame1 w.r.t. frame2
  c2_t_c1.setZero();	// The unit translation vector of frame1 w.r.t. frame2
  error_tolerance = 0;  // The error tolerance measure defined to classify a feature point as an inlier
  size_rejection_percent = 0.2;
}
FivePointSolver::FivePointSolver(double _error_tolerance,
                                 double _rejection_percent = 0.5)
    : RansacSolver(_error_tolerance, _rejection_percent) {
  min_set_size = 5;
  solution_type = 5;
  c2_R_c1.setIdentity();  // The quaternion representing the orientation of frame1 w.r.t. frame2
  c2_t_c1.setZero();	// The unit translation vector of frame1 w.r.t. frame2
}
/// Destructor
FivePointSolver::~FivePointSolver() {
}

//Any minimal solver will get the indexes for generating a model from here.
void FivePointSolver::SolveMinimal(const std::vector<int>& indexes) {
  Eigen::Matrix<double, 3, 5> sample1;
  Eigen::Matrix<double, 3, 5> sample2;
  for (unsigned int i = indexes.size(); i--;) {
    sample1.col(i) = measurements_frame1.col(indexes[i]);
    sample2.col(i) = measurements_frame2.col(indexes[i]);
  }
  Geometry::FivePointAlgorithm(sample1, sample2, c2_R_c1, c2_t_c1);
}
//Any minimal solver will push the indexes in here.
void FivePointSolver::GetInliers(std::vector<int> &inlier_index,
                                 std::vector<int> &outlier_index) {
  inlier_index.clear();
  outlier_index.clear();
  int outliers_count = 0;
//   int outliers_bound = size_rejection_percent * data_size;
  outlier_index.clear();
  inlier_index.clear();
  for (unsigned int i = 0; i < data_size; i++) {
    Eigen::Vector3d point1 = measurements_frame1.col(i);
    Eigen::Vector3d point2 = measurements_frame2.col(i);
    Eigen::Matrix3d essentialMatrix = Geometry::SkewSymmetricMatrix(c2_t_c1)
        * c2_R_c1;
    Eigen::Vector3d bearing1 = point1 / point1.norm();
    Eigen::Vector3d bearing2 = point2 / point2.norm();

    double error = Geometry::SampsonEpipolarError(bearing1, bearing2, essentialMatrix);
    Eigen::Vector2d depths = Geometry::EstimateDepth(point1, point2, c2_R_c1, c2_t_c1);
    bool behind_cam = false;
    if(depths(0) < 0 || depths(1) < 0) {
      behind_cam = true;
    }
    if (error < error_tolerance
         && !behind_cam) {
      inlier_index.push_back(i);
    } else {
      outlier_index.push_back(i);
      outliers_count++;
    }
  }
}
} /** End of namespace: Ransac */
