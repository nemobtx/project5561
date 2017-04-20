#ifndef THREE_POINT_PLUS_GRAVITY_THREE_POINT_PLUS_GRAVITY_H_
#define THREE_POINT_PLUS_GRAVITY_THREE_POINT_PLUS_GRAVITY_H_

#include <math.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <aligned-allocation.h>

#include "givens.h"
#include "quartic.h"
namespace Geometry {
bool Solve3PointPlusGravityEssential(
    const std::vector<double> &points1, const std::vector<double> &points2,
    const Eigen::Vector3d &direction_1, const Eigen::Vector3d &direction_2,
    bool use_companion_matrix,
    MARS::Aligned<std::vector, Eigen::Matrix<double, 3, 3> >::type* estimated_E,
    MARS::Aligned<std::vector, Eigen::Matrix<double, 3, 4> >::type* estimated_P);

bool Solve3PointPlusGravityEssentialPureRotation(
    const std::vector<double> &points1, const std::vector<double> &points2,
    const Eigen::Vector3d &direction_1, const Eigen::Vector3d &direction_2,
    bool use_companion_matrix,
    MARS::Aligned<std::vector, Eigen::Matrix<double, 3, 3> >::type* estimated_E,
    MARS::Aligned<std::vector, Eigen::Matrix<double, 3, 4> >::type* estimated_P);

bool Solve3PointPlusGravityEssentialImpl(
    const std::vector<double> &points1, const std::vector<double> &points2,
    const Eigen::Vector3d &direction_1, const Eigen::Vector3d &direction_2,
    bool use_companion_matrix, bool zero_translation_case,
    MARS::Aligned<std::vector, Eigen::Matrix<double, 3, 3> >::type *estimated_E,
    MARS::Aligned<std::vector, Eigen::Matrix<double, 3, 4> >::type *estimated_P);
} // namespace Geometry
#endif  // THREE_POINT_PLUS_GRAVITY_THREE_POINT_PLUS_GRAVITY_H_
