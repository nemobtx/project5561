#ifndef TWO_PT_RS_SOLVER_H
#define TWO_PT_RS_SOLVER_H

#include <Eigen/Core> 
#include <Eigen/Geometry>
#include "ransac_solver.h"
#include "robotics3d/numerics.h"

class two_pt_rs_solver : public ransac_solver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  two_pt_rs_solver() {
  }
  ;
  virtual ~two_pt_rs_solver() {
  }
  ;

  //
  // Overriden functions
  //

  void prepare_frames();
  void prepare_features();
  void leave_frames();
  void leave_features();

  void generateHypothesis(std::vector<int> indexes_of_random_sample);
  void getInliers(std::vector<int> &inlier_index,
                  std::vector<int> &outlier_index);
  int getSolutionType();
  int getMinimumSetSize() {
    int minimum_set = 2;
    return minimum_set;
  }
  ;

  void setAlgebraicErrorTolerance(double _algebraic_error_tolerance) {
    algebraic_error_tolerance = _algebraic_error_tolerance;
  }
  ;
  void setSampsonErrorTolerance(double _sampson_error_tolerance) {
    sampson_error_tolerance = _sampson_error_tolerance;
  }
  ;
  void setSufficientAngle(double _sufficient_angle) {
    sufficient_angle = _sufficient_angle;
  }
  ;

  double getSufficientAngle() {
    return sufficient_angle;
  }
  ;
  double getAlgebraicErrorTolerance() {
    return algebraic_error_tolerance;
  }
  ;
  double getSampsonErrorTolerance() {
    return sampson_error_tolerance;
  }
  ;

  void setErrorType(int _error_type) {
    error_type = _error_type;
  }
  ;
  int getErrorType() {
    return error_type;
  }
  ;

 private:
  // quaternion c2_q_c1;		
  // Eigen::Matrix3d c2_C_c1;
  // Eigen::Vector3d c2_p_c1;
  // Eigen::Vector3d c2_t_c1_scaled;
  // Eigen::Vector2d cc;
  Eigen::Matrix3d essentialMatrix;
  Eigen::Vector3d Ep1;
  Eigen::Vector3d Ep2;
// 	int solution_type; // 0 hovering 2 not hovering

  double algebraicError(int _correspondence_index);
  double sampsonError(int _correspondence_index);
  double AngleDifference(int _correspondence_index);

  double sufficient_angle;

  double algebraic_error_tolerance;
  double sampson_error_tolerance;

  int error_type;

  // 	Eigen::Matrix3d i_C_c;
  // 	Eigen::Vector3d i_p_c;

  // 	Eigen::Matrix3d ii_C_i_1;
  // 	Eigen::Matrix3d ii_C_i_2;
  // 	Eigen::Matrix3d k1_C_1;
  // 	Eigen::Matrix3d k2_C_2;
  // 	Eigen::Vector3d c1_p_ck1;
  // 	Eigen::Vector3d ck2_p_c2;
  // 	Eigen::Matrix3d ck1_C_ck2;

  // 	double rs_1;
  // 	double ts_1;
  // 	double ts_compensate_1;
  // 	double rs_2;
  // 	double ts_2;
  // 	double ts_compensate_2;
  // 	Eigen::Vector4d i_q_g_1;
  // 	Eigen::Matrix3d i_C_g_1;
  // Eigen::Vector3d b_g_1;
  // Eigen::Vector3d g_v_i_1;
  // Eigen::Vector3d w_1;
  // Eigen::Vector4d i_q_g_2;
  // Eigen::Matrix3d i_C_g_2;
  // Eigen::Vector3d b_g_2;
  // Eigen::Vector3d g_v_i_2;
  // Eigen::Vector3d w_2;
  // double imu_dt;
  // int camera_dimension_y;
  // Eigen::Vector3d imu_rot;
  // Eigen::Vector4d imu_rot_quat;

  Eigen::Vector3d A_row;
  double b_element;
  Eigen::Matrix3d A_RS;
  Eigen::Vector3d b_RS;
};

#endif
