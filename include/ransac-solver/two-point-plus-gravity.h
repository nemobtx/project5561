// Copyright 2013 Motorola Mobility LLC. Part of the Trailmix project.
// CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
// Authors: Dimitris Kottas (dkottas@cs.umn.edu),
//          Ryan DuToit (dutoi006@umn.edu)
#ifndef TWO_POINT_PLUS_GRAVITY_TWO_POINT_PLUS_GRAVITY_H_
#define TWO_POINT_PLUS_GRAVITY_TWO_POINT_PLUS_GRAVITY_H_

#include <vector>

#include <Eigen/Core>
#include <aligned-allocation.h>

// Solve2PointPlusGravity Solves the two point plus gravity problem.
// Inputs:
// pts_camera = Two (2) homogeneous point coordinates in camera frame.
// pts_global = Two (2) global point coordinates.
// C_gravity_normalized = Normalized gravity measurement in camera frame
//                        assuming Global frame has gravity = [0, 0, 1]^T.
// Outputs:
// estimated_P = Two (2) 3x4 matrices that solve the solution, [R | t]
// The user must choose the correct of the two.
bool Solve2PointPlusGravity(
    const std::vector<double>& pts_camera,
    const std::vector<double>& pts_global,
    const Eigen::Matrix<double, 3, 1>& C_gravity_normalized,
    MARS::Aligned<std::vector, Eigen::Matrix<double, 3, 3> >::type* estimated_E,
    MARS::Aligned<std::vector, Eigen::Matrix<double, 3, 4> >::type* estimated_P);

#endif  // TWO_POINT_PLUS_GRAVITY_TWO_POINT_PLUS_GRAVITY_H_
