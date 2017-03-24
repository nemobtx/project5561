#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

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
   
   ORB orb;
   int minHessian = 400;
   OrbFeatureDetector detector(minHessian);
   //OrbFeatureDetector detector(25, 1.0f, 2, 10, 0, 2, 0, 10);
   OrbDescriptorExtractor extractor;
   vector<KeyPoint> keyp1, keyp2;
   orb.detect(im1, keyp1);
   orb.detect(im2, keyp2);
   orb.compute(im1, keyp1, des1);
   orb.compute(im2, keyp2, des2);
   
  namedWindow("Display Image", WINDOW_AUTOSIZE );
  Mat imDraw;
  drawKeypoints(im1, keyp1, imDraw);
  
  imshow("Display Image", imDraw);
  waitKey(0);
  return 0;
}