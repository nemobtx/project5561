#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  if ( argc != 3 ){
    printf("usage: DisplayImage.out <Image1_Path> <Image2_Path>\n");
    return -1;
  }
  Mat im1=imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  Mat im2=imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE); 
  if ( (!im1.data || !im2.data) ){
    printf("No image data \n");
    return -1;
   }
   Mat des1, des2;
   
   Ptr<FeatureDetector> detector = ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::FAST_SCORE, 31, 100);
   //int minHessian = 400;
   //detector->setHessianThreshold(minHessian);
   //FeatureDetector detector(minHessian);
   //OrbFeatureDetector detector(25, 1.0f, 2, 10, 0, 2, 0, 10);
   Ptr<DescriptorExtractor> extractor = ORB::create();
   vector<KeyPoint> keyp1, keyp2;
   detector->detect(im1, keyp1);
   detector->detect(im2, keyp2);
   extractor->compute(im1, keyp1, des1);
   extractor->compute(im2, keyp2, des2);
   
  namedWindow("Image 1", WINDOW_AUTOSIZE );
  namedWindow("Image 2", WINDOW_AUTOSIZE );
  Mat imDraw1, imDraw2;
  drawKeypoints(im1, keyp1, imDraw1);
  drawKeypoints(im2, keyp2, imDraw2);
  imshow("Image 1", imDraw1);
  imshow("Image 2", imDraw2);
  
  cout << "numFeature1 "<< keyp1.size();
  cout << " numFeature2 "<< keyp2.size()<<endl;
   // greedy matches
   FlannBasedMatcher matcher;
   std::vector<DMatch> matches;
   if(des1.type()!=CV_32F) des1.convertTo(des1, CV_32F);
   if(des2.type()!=CV_32F) des2.convertTo(des2, CV_32F);
   
   matcher.match(des1, des2, matches);
   double max_dist = 0; double min_dist = 100;
  
   cout << "found "<<matches.size()<< " matches"<<endl;
   //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < des1.rows; i++ ){
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  
  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;
  for( int i = 0; i < des1.rows; i++ ) { 
//    if( matches[i].distance <= max(2*min_dist, 0.02) ){
  if( matches[i].distance <= 100){
    good_matches.push_back( matches[i]); }
  }
  //-- Draw only "good" matches
  
  cout << "drawing good matches...";
  cout << good_matches.size()<< " good matches"<<endl; 
  Mat img_matches;
  drawMatches( im1, keyp1, im2, keyp2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
   
  //show detected matches
  imshow("Good Matches", img_matches);
  /*
  for( int i = 0; i < (int)good_matches.size(); i++ ){
    printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n",
	    i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
  }*/

  waitKey(0);
  sleep(20000);
  return 0;
}
