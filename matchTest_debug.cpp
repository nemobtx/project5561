//
// Created by Kat on 4/26/17.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "utils.h"
#include <string>
#include <stdlib.h>
//printing
#include <fstream>
#include <iomanip>
#include "OrbMatching.hpp"

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
    double rThres = 0.00003;
    float distThres = 7;
    int rIter = 5000;
    int distance_threshold = 40;
    Mat im1, im2;
    int startInd=0;
    int endInd = 31;
    startInd = atoi(argv[1]);
    endInd = atoi(argv[2]);

    if (argc < 3) {
        printf("usage: ./orbDebug <Image1_Path> <Image2_Path> <ransac iter> <ransac thres> <maxsuppress dist>\n");
        return -1;
    } else if (argc == 4) {
        rIter = atoi(argv[3]);
    } else if (argc == 5) {
        rIter = atoi(argv[3]);
        rThres = atof(argv[4]);
    } else if (argc == 6) {
        rIter = atoi(argv[3]);
        rThres = atof(argv[4]);
        distThres = atof(argv[5]);
    }
    im1 = cv::imread(argv[1]);
    im2 = cv::imread(argv[2]);
    if ((!im1.data || !im2.data)) {
        printf("No image data \n");
        return -1;
    }
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
    p1.fc = fc1;
    p1.kc = kc1;
    p1.cc = cc1;
    p1.width = 640;
    p1.height = 480;
    p1.fisheye = true;
    p6.fc = fc1;
    p6.kc = kc1;
    p6.cc = cc1;
    p6.width = 640;
    p6.height = 480;
    p6.fisheye = true;
    Mat im1_un, im2_un;
    undist(im1, im1_un, p1);
    undist(im2, im2_un, p6);
    //imshow("undistort1", im1_un);
    //imshow("undistort2", im2_un);
    
    ///rotate 90///
    Mat im1_rot, im2_rot;
    transpose(im1_un, im1_rot);
    transpose(im2_un, im2_rot);
    //imshow("trans1", im1_rot);
    //imshow("trans2", im2_rot);
    
    flip(im1_rot, im1_rot, 1);
    flip(im2_rot, im2_rot, 1);
    //imshow("flip1", im1_rot);
    //imshow("flip2", im2_rot);

    ORBMatching ob;
    ob.distance_threshold = distance_threshold;

    Mat des1, des2, des1_temp, des2_temp;
    vector<KeyPoint> keyp1, keyp2, keyp1_temp, keyp2_temp;
    //float distThres = 5;
    ob.findFeatures(im1_rot, des1_temp, keyp1_temp);
    ob.findFeatures(im2_rot, des2_temp, keyp2_temp);
    ob.drawFeatures(im1_rot, keyp1_temp, "features for im1");
    ob.drawFeatures(im2_rot, keyp2_temp, "features for im2");
    //cout << "num keypoints for im1: "<< keyp1_temp.size()<<", im2: "<<keyp2_temp.size()<<endl;
    //cout << "descriptor size for im1: " << des1_temp.size().height<< " " << des1_temp.size().width;
    //cout << ", for im2: " << des2_temp.size().height<< " " << des2_temp.size().width<<endl;

    ob.maxSuppresss(keyp1_temp, des1_temp, keyp1, des1, distThres);
    ob.maxSuppresss(keyp2_temp, des2_temp, keyp2, des2, distThres);
    ob.drawFeatures(im1_rot, keyp1, "max suppresed features im1");
    ob.drawFeatures(im2_rot, keyp2, "max suppresed features im2");
    //cout << "num keypoints for im1: "<< keyp1.size()<<", im2: "<<keyp2.size()<<endl;
    //cout << "descriptor size for im1: " << des1.size().height<< " " << des1.size().width;
    //cout << ", for im2: " << des2.size().height<< " " << des2.size().width<<endl;


    //BF Matches
    vector<DMatch> matches, inlierMatches;
    ob.matchFeatures(des1, des2, keyp1, keyp2, matches);
    ob.drawORBmatches(im1_rot, im2_rot, keyp1, keyp2, matches, "BF matches");
    
    if (matches.size() <= 5){
      cout <<"matches less than 6, no ransac"<<endl;
      im1.release();
      im2.release();
      waitKey(0);
      sleep(2000);
      return 0;
    } else {
      // RANSAC
      srand(time(NULL));
      ob.numIter = rIter;
      ob.thresVal = rThres;
      cout << "Ransac iter: " << ob.numIter << " threshold: " << ob.thresVal << endl;
      Eigen::Matrix3d Kinv1, Kinv2;
      Kinv1 << 1/p1.fc[0],0,-p1.cc[0]/p1.fc[0],
	    0,1/p1.fc[1],-p1.cc[1]/p1.fc[1],
	    0,0,1;
      Kinv2 << 1/p6.fc[0],0,-p6.cc[0]/p6.fc[0],
	    0,1/p6.fc[1],-p6.cc[1]/p6.fc[1],
	    0,0,1;
      /*
      Kinv1 << 0.0039, 0, -0.9538,
      0, 0.0039, -1.2338,
      0, 0, 1;
      Kinv2 << 0.0039, 0, -0.9194,
      0, 0.0039, -1.2815,
      0, 0, 1;
      */
      ob.fivePointInlier(keyp1, keyp2, Kinv1, Kinv2, matches, inlierMatches);
      ob.drawORBmatches(im1_rot, im2_rot, keyp1, keyp2, inlierMatches, "inlier matches");
      im1.release();
      im2.release();      
      waitKey(0);
      sleep(2000);

      return 0;
    }
}
