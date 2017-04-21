#include <iostream>
#include <opencv2/opencv.hpp>
//#include <unistd.h>
#include "include/imgproc/orb/orbextractor.h"
//#include "include/imgproc/freak/freak.h"
#include <Eigen/Dense>
#include "ransac-solver/five-point-solver.h"

using namespace cv;
using namespace std;
using namespace Ransac;
using namespace Eigen;

int main(int argc, char** argv) {
  if ( argc != 4 ){
    printf("usage: DisplayImage.out <Image1_Path> <Image2_Path> extractorType\n");
    return -1;
  }
  Mat im1=imread( argv[1]);cvtColor(im1, im1, cv::COLOR_RGB2GRAY);
  Mat im2=imread( argv[2]);cvtColor(im2, im2, cv::COLOR_RGB2GRAY);
  bool USE_ORB = argv[3];
  if ( (!im1.data || !im2.data) ){
    printf("No image data \n");
    return -1;
  }
  Mat des1, des2;
  vector<cv::KeyPoint> keyp1, keyp2;
  unsigned int descriptor_size = 0, distance_threshold = 0, score_threshold = 0;
  
  if (USE_ORB) {
    descriptor_size = 32;
    distance_threshold = 50;
    score_threshold = 205;
    // max_features, scale_factor,pyramid_level, edge_threshold, wta_k, patch_size
    EyeMARS::orbextractor orb_extractor(200, 1.2f, 3, 31, 4, 31);

    orb_extractor.ExtractKeypointsDescriptors(im1, keyp1, des1);
    orb_extractor.ExtractKeypointsDescriptors(im2, keyp2, des2);
  } else {
      // freak
   descriptor_size = 64;
   distance_threshold = 100;
   score_threshold = 480;
 //  EyeMARS::FREAKExtractor freak_extractor;
   
 //  freak_extractor.FindAndExtractFreakFeatures(im1, keyp1, des1);
  // freak_extractor.FindAndExtractFreakFeatures(im2, keyp2, des2);
 }
 
 namedWindow("Image 1", WINDOW_AUTOSIZE );
 namedWindow("Image 2", WINDOW_AUTOSIZE );
 Mat imDraw1, imDraw2;
 drawKeypoints(im1, keyp1, imDraw1);
 drawKeypoints(im2, keyp2, imDraw2);
 imshow("Image 1", imDraw1);
 imshow("Image 2", imDraw2);
 
 cout << "numFeature1 "<< keyp1.size();
 cout <<" numFeature2 "<< keyp2.size()<<endl;


 
 
 
  // Find matches
 cv::BFMatcher matcher(cv::NORM_HAMMING);
 std::vector<cv::DMatch> matches;
 if(des1.type()!=CV_8U) des1.convertTo(des1, CV_8U);
 if(des2.type()!=CV_8U) des2.convertTo(des2, CV_8U);
 matcher.match(des1, des2, matches);
 float max_dist = 0; float min_dist = 150;
 
 cout << "found "<<matches.size()<< " matches"<<endl;
   //-- Quick calculation of max and min distances between keypoints
 for( int i = 0; i < des1.rows; i++ ){
  float dist = matches[i].distance;
  if( dist < min_dist ) min_dist = dist;
  if( dist > max_dist ) max_dist = dist;
}
printf("-- Max dist : %f \n", max_dist );
printf("-- Min dist : %f \n", min_dist );

std::vector< DMatch > good_matches;
for( int i = 0; i < des1.rows; i++ ) { 
  if( matches[i].distance <= distance_threshold){
    good_matches.push_back( matches[i]); }
  }
  //-- Draw only "good" matches
  cout << "drawing good matches...";
  cout << good_matches.size()<< " good matches"<<endl; 
  Mat img_matches;
  int numMatches = good_matches.size();
  drawMatches( im1, keyp1, im2, keyp2,
   good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  //cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
  //show detected matches
  imshow("Good Matches", img_matches);
  

  // Inlier Detection
  Eigen::Matrix3f K, Kinv;
  K << 256.474, 0, 328.671,
	0,256.487, 235.822,
	0,0,1;
  Kinv << 0.0039, 0, -1.2815,
	  0, 0.0039, -0.9194,
	  0,0,1;
  FivePointSolver solver;//mars lab 5 point RANSAC
  MatrixXd measurements_frame1, measurements_frame2;
  measurements_frame1.resize(3, numMatches);
  measurements_frame2.resize(3, numMatches);
  

  for (int i=0; i<numMatches; ++i){
    measurements_frame1(0,i) = Kinv(0,0)*keyp1[i].pt.x+Kinv(0,1)*keyp1[i].pt.y+Kinv(0,2);
    measurements_frame1(1,i) = Kinv(1,0)*keyp1[i].pt.x+Kinv(1,1)*keyp1[i].pt.y+Kinv(1,2);
    measurements_frame1(2,i) = Kinv(2,0)*keyp1[i].pt.x+Kinv(2,1)*keyp1[i].pt.y+Kinv(2,2);
    measurements_frame2(0,i) = Kinv(0,0)*keyp2[i].pt.x+Kinv(0,1)*keyp2[i].pt.y+Kinv(0,2);
    measurements_frame2(1,i) = Kinv(1,0)*keyp2[i].pt.x+Kinv(1,1)*keyp2[i].pt.y+Kinv(1,2);
    measurements_frame2(2,i) = Kinv(2,0)*keyp2[i].pt.x+Kinv(2,1)*keyp2[i].pt.y+Kinv(2,2);
    
  }
  cout << measurements_frame1.rows() << " " << measurements_frame1.cols() << endl;
  solver.setMeasurements(measurements_frame1, measurements_frame2);
  vector<int> inlier_index, outlier_index;
  solver.GetInliers(inlier_index, outlier_index);
  cout << inlier_index.size()<< " inliers and "<< outlier_index.size() << " outliers"<<endl; 
  std::vector< DMatch > inlier_matches;
  for (auto i:inlier_index) {
    cout << i << " ";
    inlier_matches.push_back( good_matches[i]); 
  }
  Mat in_matches;
  drawMatches( im1, keyp1, im2, keyp2,
   inlier_matches, in_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  //cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
  //show detected matches
  imshow("inlier Matches", in_matches);
  
  
  
  waitKey(0);
  sleep(20000);
  return 0;
}
