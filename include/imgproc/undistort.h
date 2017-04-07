#ifndef EYEMARS_IMGPROC_UNDISTORT_H_
#define EYEMARS_IMGPROC_UNDISTORT_H_

#include <demo-parameters/parameters.h>


#include <types.h>
#include <geometry/camera-models.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <demo-parameters/parameters.h>


namespace EyeMARS {
  typedef struct {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2d fc;
    Eigen::Matrix<double, 5, 1> kc;
    Eigen::Matrix<double, 2, 1> cc;
    bool fisheye = 0;
    int width = Parameters::instance()->camera_dimension_x;
    int height = Parameters::instance()->camera_dimension_y;
  } CameraParameters;

  typedef struct {
    cv::Mat distorted_points_x;
    cv::Mat distorted_points_y;
  } UndistortMap;

  UndistortMap* ComputeMap(CameraParameters const &);

  Image* UndistortImage(Image*, CameraParameters const &);

  Image* UndistortImage(Image*, UndistortMap*);
};

#endif  // EYEMARS_IMGPROC_UNDISTORT_H_
