//
// Created by tong on 4/7/17.
//
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "imgproc/orb/orbextractor.h"
#include "ransac-solver/five-point-solver.h"

using namespace std;
using namespace Ransac;
using namespace Eigen;

int main() {
/*  EyeMARS::orbextractor orb_extractor;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  cv::Mat img = cv::imread("imgs/1.jpg");
  cvtColor(img, img, cv::COLOR_RGB2GRAY);
  orb_extractor.ExtractKeypointsDescriptors(img, keypoints, descriptors);
  cv::Mat img2;
  cv::equalizeHist(img, img2);
  cv::imshow("1", img2);
  cv::waitKey();*/
  FivePointSolver solver;
  MatrixXd measurements_frame1, measurements_frame2;
  measurements_frame1.resize(3, 10);
  measurements_frame2.resize(3, 10);
  measurements_frame1.setRandom();
  measurements_frame2.setRandom();
  solver.setMeasurements(measurements_frame1, measurements_frame2);
  vector<int> inlier_index, outlier_index;
  solver.GetInliers(inlier_index, outlier_index);
  for (auto i:inlier_index) {
    cout << i << " ";
  }
  cout << outlier_index.size() << endl;
  return 0;
}

