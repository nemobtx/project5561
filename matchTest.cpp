//
// Created by Kat on 4/26/17.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "utils.h"
#include "OrbMatching.hpp"

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  if ( argc != 4 ){
    printf("usage: DisplayImage.out <Image1_Path> <Image2_Path> extractorType\n");
    return -1;
  }
  Mat im1=cv::imread(argv[1]);
  Mat im2=cv::imread(argv[2]);
  cvtColor(im1, im1, COLOR_RGB2GRAY);
  cvtColor(im2, im2, COLOR_RGB2GRAY);
  // undistort the images
  Vector2d fc1, cc1, fc6, cc6;
  Matrix<double, 5, 1> kc1, kc6;
  fc1 << 255.528, 255.556;
  cc1 << 315.271, 243.758;
  kc1 << 0.921633, .0, .0, .0, .0;
  fc6 << 256.474, 256.487;   
  cc6 << 328.671, 235.822;
  kc6 << 0.920773, .0, .0, .0, .0;
  EyeMARS::CameraParameters p1, p6;
  p1.fc = fc1; p1.kc = kc1; p1.cc = cc1;
  p1.width = 640; p1.height = 480;
  p1.fisheye = true;
  p6.fc = fc1; p6.kc = kc1; p6.cc = cc1;
  p6.width = 640; p6.height = 480;
  p6.fisheye = true;
  Mat im1_un, im2_un;
  undist(im1, im1_un, p1);
  undist(im2, im2_un, p6);
  
  bool USE_ORB = argv[3];
  if ( (!im1.data || !im2.data) ){
    printf("No image data \n");
    return -1;
  }
  //rotate 90
  Mat im1_rot, im2_rot;
  transpose(im1_un, im1_rot);flip(im1_rot, im1_rot,0);
  transpose(im2_un, im2_rot);flip(im2_rot, im2_rot,0);
  
  ORBMatching ob;
  ob.distance_threshold = 20;
  //5 point parameters
  ob.numIter = 500; ob.thresVal = 0.5;
  
  Mat des1,des2;  vector<KeyPoint> keyp1, keyp2;
  ob.findFeatures(im1_rot, des1, keyp1);
  ob.findFeatures(im2_rot, des2, keyp2);
  vector<DMatch> matches, inlierMatches;
  ob.matchFeatures(im1_rot, im2_rot, des1, des2, keyp1, keyp2, matches);
  
  cout <<"Ransac iter: "<<ob.numIter << " threshold: "<<ob.thresVal<<endl;
  Eigen::Matrix3f Kinv1, Kinv2;
  //Kinv1 << 1,0,0,0,1,0,0,0,1;
  //Kinv2 =Kinv1;
  Kinv1 << 0.0039, 0, -0.9538,
	  0, 0.0039, -1.2338,
	  0,0,1;
  Kinv2 << 0.0039, 0, -0.9194,
	  0, 0.0039, -1.2815,
	  0,0,1;
   ob.fivePointInlier(keyp1, keyp2, Kinv1, Kinv2, matches, inlierMatches);
   Mat in_matches;
   drawMatches( im1_rot, keyp1, im2_rot, keyp2,
	    inlierMatches, in_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
   imshow("inlier Matches", in_matches);
  //cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
  
    waitKey(0);
  sleep(20000);
   
  return 0;
}
