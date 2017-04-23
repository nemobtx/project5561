//
// Created by tong on 4/7/17.
//
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "imgproc/orb/orbextractor.h"
#include "ransac-solver/five-point-solver.h"
#include "utils.h"

using namespace std;
using namespace Ransac;
using namespace Eigen;
using namespace cv;
using namespace Geometry;

int main() {
/*  FivePointSolver solver;
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
  cout << outlier_index.size() << endl;*/
  Vector2d fc, cc;
  Matrix<double, 5, 1> kc;
  fc << 255.528, 255.556;
  cc << 315.271, 243.758;
  kc << 0.921633, .0, .0, .0, .0;
  cv::Mat img1 = cv::imread("imgs/exposure0/1.jpg", cv::IMREAD_GRAYSCALE);
  imshow("ha2", img1);

  EyeMARS::CameraParameters p1;
  p1.fc = fc;
  p1.kc = kc;
  p1.cc = cc;
  p1.width = 640;
  p1.height = 480;
  p1.fisheye = true;
  
  Mat img1_undist;
  undist(img1, img1_undist, p1);
  imshow("ha", img1_undist);
  waitKey();
  return 0;
}

