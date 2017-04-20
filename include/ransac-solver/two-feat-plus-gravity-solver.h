#ifndef TWO_FEAT_PLUS_GRAVITY_SOLVER_H_
#define TWO_FEAT_PLUS_GRAVITY_SOLVER_H_

#include <Eigen/Core> 
#include <Eigen/Geometry>

#include "ransac-solver/ransac-solver.h"	/// To use: RansacSolver
#include <geometry/geometry.h>
#include <camera-intrinsics-models.h>
#include <aligned-allocation.h>
/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
/**	@class TwoFeatPlusGravitySolver
 * 	@brief The TwoFeatPlusGravitySolver class.
 */
class TwoFeatPlusGravitySolver : public RansacSolver {
 protected:
  // Data Members
  Eigen::VectorXd *xkk_1_;
  Eigen::VectorXd *xkk_2_;
  Eigen::VectorXd *xkk_land_1_;
  Eigen::VectorXd *xkk_land_2_;
  Eigen::VectorXd *landmark_first_pose_1_;
  Eigen::VectorXd *landmark_first_pose_2_;
  std::vector<int> *map_landmarks_1_;
  std::vector<int> *map_landmarks_2_;
  std::vector<int> *common_feat_ids_;
  Eigen::Vector3d i_p_c_1;
  Eigen::Vector3d i_p_c_2;
  Eigen::Vector4d i_q_c_1;
  Eigen::Vector4d i_q_c_2;
  Eigen::Vector3d G2_p_G1;
  double sin_theta;
  double cos_theta;
  int state_size_1_;
  int state_size_2_;


 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TwoFeatPlusGravitySolver(double _error_tolerance,
                           double _rejection_percent = 0.5)  
    : RansacSolver(_error_tolerance, _rejection_percent){
    min_set_size = 2;
  }
  ~TwoFeatPlusGravitySolver(){}
  void set_state_size(int _state_size_1, int _state_size_2) {
    state_size_1_ = _state_size_1;
    state_size_2_ = _state_size_2;
  }

  void set_xkk(Eigen::VectorXd &_xkk_1, Eigen::VectorXd &_xkk_2)  {
    xkk_1_ = &_xkk_1;
    xkk_2_ = &_xkk_2;
  }
  void set_xkk_land(Eigen::VectorXd &_xkk_land_1, Eigen::VectorXd &_xkk_land_2)  {
    xkk_land_1_ = &_xkk_land_1;
    xkk_land_2_ = &_xkk_land_2;
  }
  void set_landmark_first_pose(Eigen::VectorXd &_landmark_first_pose_1,
                               Eigen::VectorXd &_landmark_first_pose_2)  {
    landmark_first_pose_1_ = &_landmark_first_pose_1;
    landmark_first_pose_2_ = &_landmark_first_pose_2;
  }
  void set_map_landmarks(std::vector<int> &_map_landmarks_1,
                         std::vector<int> &_map_landmarks_2) {
    map_landmarks_1_ = &_map_landmarks_1;
    map_landmarks_2_ = &_map_landmarks_2;
  }
  void set_common_feat_ids(std::vector<int> &_common_feat_ids)  {
    common_feat_ids_ = &_common_feat_ids;
    data_size = _common_feat_ids.size();
  }
  void set_camera_extrinsic(Eigen::Vector3d _i_p_c_1, Eigen::Vector4d _i_q_c_1, 
                            Eigen::Vector3d _i_p_c_2, Eigen::Vector4d _i_q_c_2)  {
    i_p_c_1 = _i_p_c_1;
    i_p_c_2 = _i_p_c_2;
    i_q_c_1 = _i_q_c_1;
    i_q_c_2 = _i_q_c_2;
  }


  void SolveMinimal(const std::vector<int>& indexes);
  void GetInliers(std::vector<int> &inlier_index, std::vector<int> &outlier_index);

};
/** End of class: TwoFeatPlusGravitySolver */
} /** End of namespace: Ransac */
#endif	/** End of file: TWO_FEAT_PLUS_GRAVITY_SOLVER_H_ */
