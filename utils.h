//
// Created by tong on 4/21/17.
//

#ifndef PROJECT5561_UTILS_H
#define PROJECT5561_UTILS_H

#include <opencv2/opencv.hpp>
#include "imgproc/undistort.h"

void undist(const cv::Mat &distorted, cv::Mat &undistorted, EyeMARS::CameraParameters parameters) {
  cv::Mat distorted1 = distorted.clone();
  EyeMARS::Image image(distorted1);
  EyeMARS::Image *image_undist = EyeMARS::UndistortImage(&image, parameters);
  undistorted = image_undist->CvMat();
}

#endif //PROJECT5561_UTILS_H
