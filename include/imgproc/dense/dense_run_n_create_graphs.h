/*
 * dense_run_n_create_graphs.h
 *
 *  Created on: Apr 13, 2015
 *      Author: mars
 */

#ifndef SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_DENSE_RUN_N_CREATE_GRAPHS_H_
#define SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_DENSE_RUN_N_CREATE_GRAPHS_H_


#include <iostream>
#include <stdio.h>
#include "imgproc/dense/config.h"
#include "imgproc/dense/observe_dense.h"
#include "imgproc/dense/gradients.h"
#include <string.h>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "core/image.h"
#include "core/matrix2d.h"
#include "geometry/geometry.h"
#include <load-from-file.h>
#include "tictoc.h"

namespace EyeMARS {

void dense_run_n_create_graphs(std::vector<std::vector <double>> &new_features, std::vector <size_t> &feature_cnt, Dense_Params &parameters);
}

#endif /* SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_DENSE_RUN_N_CREATE_GRAPHS_H_ */
