//
// Created by katherine on 4/26/17.
//

#ifndef PROJECT5561_ORBMATCHING_H
#define PROJECT5561_ORBMATCHING_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
//#include "utils.h"

class ORBMatching {
 public:
   //matching parameters
  int distance_threshold = 20;
  //5 point parameters
  int numIter = 500;double thresVal = 0.5;
  void findFeatures(cv::Mat& im, cv::Mat& des, std::vector<cv::KeyPoint>& keyp);
  void matchFeatures(cv::Mat& im1, cv::Mat& im2, cv::Mat& des1, cv::Mat& des2, std::vector<cv::KeyPoint>& keyp1, std::vector<cv::KeyPoint>& keyp2, std::vector<cv::DMatch>& good_matches);
  void fivePointInlier(std::vector<cv::KeyPoint>& keyp1, std::vector<cv::KeyPoint>& keyp2, Eigen::Matrix3f& Kinv1, Eigen::Matrix3f& Kinv2, std::vector<cv::DMatch>& matches, std::vector<cv::DMatch>& inlier_matches);
};


#endif //PROJECT5561_HISTMATCHING_H
