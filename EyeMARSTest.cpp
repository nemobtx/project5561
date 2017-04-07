//
// Created by tong on 4/7/17.
//
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "imgproc/orb/orbextractor.h"

using namespace std;

int main() {
  EyeMARS::orbextractor orb_extractor;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  cv::Mat img = cv::imread("imgs/1.jpg");
  orb_extractor.ExtractKeypointsDescriptors(img, keypoints, descriptors);
/*  cv::FastFeatureDetector detector;
  cout << detector.empty();*/
  return 0;
}

