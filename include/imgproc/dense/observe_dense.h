/*
 * observe_dense.h
 *
 *  Created on: Mar 30, 2015
 *      Author: mars
 */

#ifndef SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_OBSERVE_DENSE_H_
#define SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_OBSERVE_DENSE_H_

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>
#include "imgproc/dense/config.h"
#include "core/image.h"
#include "core/matrix2d.h"

namespace EyeMARS {

typedef struct {
  std::string img_path;
  std::string ba_xkk_path;
  bool is_ys;

  Eigen::Vector4d i_q_c;
  Eigen::Vector3d i_p_c;
  Eigen::Vector2d fc;
  Eigen::Vector2d cc;
  Eigen::Matrix<double, 5, 1> kc;

  double min_grad = 5;
  double var_thres =  0.25;
  double var_filter_thres = 0.008;
  double convexity_thres = 5;
  double diff_thres = 0.004;
  double diff_ratio = 0.5;
  double near_border = 1;

  int img_test_init;
  int max_img_num;
  int ba_start_ind;
  bool do_var_filt;
} Dense_Params;

void observe_dense(Image *ref_img, Image *KF_img, double *KF_gx, double *KF_gy,
    Eigen::Vector3d &cr_p_ckf, Eigen::Matrix3d &cr_R_ckf, Eigen::Vector3d &ckf_p_cr,
    std::vector<std::vector<double>> &curr_tracked_features,
    std::vector<std::vector<double>> &curr_new_feature_candidatess, bool isFirst, Dense_Params &parameters);

}


#endif /* SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_OBSERVE_DENSE_H_ */
