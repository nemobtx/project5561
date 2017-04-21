//
// Created by tong on 4/7/17.
//
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <imgproc/undistort.h>
#include "imgproc/orb/orbextractor.h"
#include "ransac-solver/five-point-solver.h"
#include "camera-models/CameraModelTango.h"
#include "imgproc/undistort.h"
#include "core/image.h"
#include "geometry/camera-models.h"

using namespace std;
using namespace Ransac;
using namespace Eigen;
using namespace cv;
using namespace Geometry;

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
//  cv::Mat img2 = cv::imread("imgs/exposure-1/1.jpg");
//  imshow("ha2", img1);
/*  MARS::CameraModelTango model(fc, cc, kc);
  Vector2d distorted, undistorted;
  distorted << 320, 240;
  model.Undistort(distorted, undistorted);
  cout << undistorted << endl;*/
//  EyeMARS::Image image2(img2);
  Vector2d fc, cc;
  Matrix<double, 5, 1> kc;
  fc << 255.528, 255.556;
  cc << 315.271, 243.758;
  kc << 0.921633, .0, .0, .0, .0;
  cv::Mat img1 = cv::imread("imgs/exposure0/1.jpg");
  EyeMARS::Image image1(img1);
  EyeMARS::CameraParameters p1;
  p1.fc = fc;
  p1.kc = kc;
  p1.cc = cc;
  p1.width = 640;
  p1.height = 480;
  p1.fisheye = true;
  EyeMARS::Image *image1_undist = EyeMARS::UndistortImage(&image1, p1);
  cv::Mat img1_undist = image1_undist->CvMat();
  imshow("ha", img1_undist);
  waitKey();
  /*
  double u, v;
  Geometry::normalizeUndistortScalarsTango(320, 240, u, v, fc, kc, cc, 20);
  cout << u << v << endl;*/
  return 0;
}

