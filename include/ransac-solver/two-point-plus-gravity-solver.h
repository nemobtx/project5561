#ifndef TWO_POINT_PLUS_GRAVITY_SOLVER_H_
#define TWO_POINT_PLUS_GRAVITY_SOLVER_H_

#include <Eigen/Core> 
#include <Eigen/Geometry>

#include "ransac-solver/ransac-solver.h"	/// To use: RansacSolver
#include <geometry/geometry.h>
#include <camera-intrinsics-models.h>
#include <aligned-allocation.h>
#include "ransac-solver/two-point-plus-gravity.h"
/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
/**	@class TwoPointPlusGravitySolver
 * 	@brief The TwoPointPlusGravitySolver class.
 *	The two point solver for the Ransac algorithm.
 */
class TwoPointPlusGravitySolver : public RansacSolver {
 private:
  double sufficient_angle;
  int error_type;
  Eigen::MatrixXd camera_points_;
  Eigen::MatrixXd global_points_;
  Eigen::Matrix<double, 3, 1> C_gravity_normalized_;
  Eigen::Matrix3d current_C_R_G;
  Eigen::Vector3d current_C_t_G;
  Eigen::Matrix<double, 2, 1> fc_;
  Eigen::Matrix<double, 5, 1> kc_;
  Eigen::Matrix<double, 2, 1> cc_;
  Eigen::Matrix<double, 3, 4> estimated_P_;
  Eigen::Matrix<double, 3, 3> estimated_E_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TwoPointPlusGravitySolver();
  TwoPointPlusGravitySolver(double _error_tolerance, double _rejection_percent,
                            double _sufficient_angle,
                            Eigen::Matrix<double, 2, 1>& cc,
                            Eigen::Matrix<double, 5, 1>& kc,
                            Eigen::Matrix<double, 2, 1>& fc,
                            Eigen::Matrix3d& _current_C_R_G,
                            Eigen::Vector3d& _current_C_t_G);

  virtual ~TwoPointPlusGravitySolver() {
  }

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

  void set_gravity_normalized(Eigen::Matrix<double, 3, 1>& gravity_normalized) {
    C_gravity_normalized_ = gravity_normalized;
  }

  void set_cc(Eigen::Matrix<double, 2, 1>& cc) {
    cc_ = cc;
  }

  void SetMeasurements(Eigen::MatrixXd* camera_points,
                       Eigen::MatrixXd* global_points) {
    std::cout << camera_points->cols() << " the number of cols" << std::endl;
    camera_points_ = *camera_points;
    global_points_ = *global_points;
    data_size =  camera_points_.cols();
    std::cout << "number of cols " << camera_points_.cols() << std::endl;
  }

  void set_current_C_R_G(Eigen::Matrix3d& current_c_r_g) {
    current_C_R_G = current_c_r_g;
  }

  void set_current_C_t_G(Eigen::Vector3d& current_c_t_g) {
    current_C_t_G = current_c_t_g;
  }

  Eigen::Matrix<double, 3, 3> estimated_E() const {
    return estimated_E_;
  }

  Eigen::Matrix<double, 3, 4> estimated_P() const {
    return estimated_P_;
  }

  void set_fc(Eigen::Matrix<double, 2, 1>& fc) {
    fc_ = fc;
  }

  void set_kc(Eigen::Matrix<double, 5, 1>& kc) {
    kc_ = kc;
  }
  unsigned int GetSetSize() {
    return camera_points_.cols();
  }
  void SolveMinimal(const std::vector<int>& indexes);
  void SolveWithInliers(const std::vector<int>& indexes_of_inliers);
  void GetInliers(std::vector<int> &inlier_index,
                  std::vector<int> &outlier_index);
};
/** End of class: TwoPointPlusGravitySolver */
} /** End of namespace: Ransac */
#endif	/** End of file: TWO_POINT_PLUS_GRAVITY_SOLVER_H_ */
