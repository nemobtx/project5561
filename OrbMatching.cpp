//
// Created by Kat on 4/26/17.
//
// 
#include <opencv2/opencv.hpp>
#include "include/imgproc/orb/orbextractor.h"
//#include <Eigen/Dense>
#include "ransac-solver/five-point-solver.h"
//#include "utils.h"
#include <math.h>
#include <string>
#include "OrbMatching.hpp"


using namespace std;
using namespace cv;
using namespace Eigen;
using namespace Ransac;

void ORBMatching::findFeatures(Mat& im, Mat& des, vector< cv::KeyPoint >& keyp) {
  //unsigned int descriptor_size = 0, distance_threshold = 0, score_threshold = 0;
  //descriptor_size = 32;
  //distance_threshold = 50;
  //score_threshold = 205;
  // max_features, scale_factor,pyramid_level, edge_threshold, wta_k, patch_size
  EyeMARS::orbextractor orb_extractor(300, 1.2f, 3, 31, 4, 31);
  orb_extractor.ExtractKeypointsDescriptors(im, keyp, des);
  //namedWindow("Image", cv::WINDOW_AUTOSIZE );
  /*
  Mat imDraw;
  drawKeypoints(im, keyp, imDraw);
  imshow("ORB features detected",imDraw);
  cout << "numFeature "<< keyp.size()<<endl;;
  */
}


void ORBMatching::matchFeatures(cv::Mat& des1, cv::Mat&des2, 
		   vector<cv::KeyPoint>& keyp1, vector<cv::KeyPoint>& keyp2,
		   vector<DMatch>& good_matches){
  // Find matches
 cv::BFMatcher matcher(cv::NORM_HAMMING, true);
 std::vector<cv::DMatch> matches;
 if(des1.type()!=CV_8U) des1.convertTo(des1, CV_8U);
 if(des2.type()!=CV_8U) des2.convertTo(des2, CV_8U);
 matcher.match(des1, des2, matches);
 //matcher.match(des1, des3, matches);
 float max_dist = 0; float min_dist = 150;
 
 cout << "found "<<matches.size()<< " matches";
 //-- Quick calculation of max and min distances between keypoints
 for( int i = 0; i < des1.rows; i++ ){
  float dist = matches[i].distance;
  if( dist < min_dist ) min_dist = dist;
  if( dist > max_dist ) max_dist = dist;
  }
  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );
  //distance_threshold = 20;
  //std::vector< DMatch > good_matches;
  for( int i = 0; i < des1.rows; i++ ) { 
    if( matches[i].distance <= distance_threshold){
      good_matches.push_back( matches[i]); }
   }
   cout <<"..."<< good_matches.size()<< 
   " good matches with distance threshold of "<< distance_threshold<<endl;
}
void ORBMatching::fivePointInlier(vector<KeyPoint>& keyp1, vector<KeyPoint>& keyp2,
		     Eigen::Matrix3d& Kinv1, Eigen::Matrix3d& Kinv2, 
		     vector<DMatch>& matches, vector<DMatch>& inlier_matches){
  int numMatches = matches.size();
  FivePointSolver solver;//mars lab 5 point RANSAC
  //int numIter = 500; int thresVal = 0.5;
  MatrixXd measurements_frame1, measurements_frame2;
  measurements_frame1.resize(3, numMatches);
  measurements_frame2.resize(3, numMatches);
  cout << numMatches<< endl;
  for (int i=0; i<numMatches; ++i){
    measurements_frame1(0,i) = Kinv1(0,0)*keyp1[i].pt.x+Kinv1(0,1)*keyp1[i].pt.y+Kinv1(0,2);
    measurements_frame1(1,i) = Kinv1(1,0)*keyp1[i].pt.x+Kinv1(1,1)*keyp1[i].pt.y+Kinv1(1,2);
    measurements_frame1(2,i) = Kinv1(2,0)*keyp1[i].pt.x+Kinv1(2,1)*keyp1[i].pt.y+Kinv1(2,2);
    measurements_frame1.col(i) = measurements_frame1.col(i)/measurements_frame1.col(i).norm();
    measurements_frame2(0,i) = Kinv2(0,0)*keyp2[i].pt.x+Kinv2(0,1)*keyp2[i].pt.y+Kinv2(0,2);
    measurements_frame2(1,i) = Kinv2(1,0)*keyp2[i].pt.x+Kinv2(1,1)*keyp2[i].pt.y+Kinv2(1,2);
    measurements_frame2(2,i) = Kinv2(2,0)*keyp2[i].pt.x+Kinv2(2,1)*keyp2[i].pt.y+Kinv2(2,2);
    measurements_frame2.col(i) = measurements_frame2.col(i)/measurements_frame2.col(i).norm();
  }
  solver.setMeasurements(measurements_frame1, measurements_frame2);
  solver.setErrorTolerance(thresVal);
  //cout << "thresVal: "<<thresVal<< " "<<numIter<<" "<<solver.getErrorTolerance()<<endl;
  
  
  vector<int> inlier_index, outlier_index, bestInlier_index, bestOutlier_index;
  set<int> selInd_set; vector<int> selInd; int currInd = 0; 
  pair<set<int>::iterator,bool> ret(selInd_set.end(), false);
  int maxIn = 0;
  for (int i=0; i<numIter; ++i){
    for(int j=0; j<6; ++j){
      while(!ret.second){
	currInd = rand()%numMatches;
	//cout << "curind "<<currInd<<endl;
	ret = selInd_set.emplace(currInd);
      }
      selInd.push_back(currInd);
      ret.second = false;
    }
    
    // solve for Essential matrix and get inliers
    solver.SolveMinimal(selInd);
    solver.GetInliers(inlier_index, outlier_index);
    cout <<"iter "<<i<< " inlier size: "<< inlier_index.size()<< endl;
    if (inlier_index.size() > maxIn){
      maxIn = inlier_index.size();
      bestInlier_index = inlier_index;
      bestOutlier_index = outlier_index;
    }
    selInd_set.clear(); selInd.clear();
  }
  cout <<"RANSAC found "<< bestInlier_index.size()<< " inliers and "<< 
           bestOutlier_index.size() << " outliers"<< endl; 
  for (auto i:bestInlier_index) {
    //cout << i << " ";
    inlier_matches.push_back(matches[i]); 
  }
  
}


void ORBMatching::maxSuppresss(vector<KeyPoint>& keyp, Mat& des,vector<KeyPoint>& keyp_new, Mat& des_new,float distThres){
  int numKP = keyp.size();
  float x_curr, y_curr, x_temp, y_temp;
  int des_r = des.rows; 
  des_new.reserve(des.rows); keyp_new.reserve(des.rows);
  vector<int> keepInd(des_r,1); 
  for(vector<KeyPoint>::iterator i = keyp.begin(); i!=keyp.end(); ++i){
    if(keepInd[i-keyp.begin()]){
      x_curr = i->pt.x;
      y_curr = i->pt.y;
      for(vector<KeyPoint>::iterator j=i; j != keyp.end(); ++j){
	if(keepInd[j-keyp.begin()]){
	  x_temp = j->pt.x;
	  y_temp = j->pt.y;
	  float dist = sqrt(pow(x_temp- x_curr, float(2.0))+pow(y_temp-y_curr, float(2.0)));
	  if(dist < distThres){
	      keepInd[j-keyp.begin()] = 0;
	  }
	}
      }
      keyp_new.push_back(*i);
      des_new.push_back(des.row(i-keyp.begin()));
    }
  }
}
  
  
void ORBMatching::drawFeatures(Mat& im, vector< cv::KeyPoint >& keyp, string name) {
  //namedWindow("Image", cv::WINDOW_AUTOSIZE );
  Mat imDraw;
  drawKeypoints(im, keyp, imDraw);
  imshow(name, imDraw);
  //imshow("ORB features detected",imDraw);
  //cout << "numFeature "<< keyp.size()<<endl;
  //sleep(20000);
}

void ORBMatching::drawORBmatches(cv::Mat& im1, cv::Mat& im2,
		vector<cv::KeyPoint>& keyp1, vector<cv::KeyPoint>& keyp2, 
		vector<DMatch>& good_matches, string name) {
  //-- Draw only "good" matches
  //cout << "drawing good matches...";
  //cout << good_matches.size()<< " good matches"<<endl; 
  Mat img_matches;
  int numMatches = good_matches.size();
  drawMatches( im1, keyp1, im2, keyp2, good_matches, img_matches,
	       cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(),
	       DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  //cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
  //show detected matches
  imshow(name, img_matches);
  //sleep(20000);

}  
  