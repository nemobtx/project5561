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
  cvtColor(img, img, cv::COLOR_RGB2GRAY);
  orb_extractor.ExtractKeypointsDescriptors(img, keypoints, descriptors);
  cv::Mat img2;
  cv::equalizeHist(img, img2);
  cv::imshow("1", img2);
  cv::waitKey();
/*  cv::FastFeatureDetector fd(20, true);
  cout << fd.empty();*/
  return 0;
}

