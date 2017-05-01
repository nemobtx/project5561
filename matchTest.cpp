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
    double rThres = 0.0001;
    float distThres = 7;
    int rIter = 10000;
    int distance_threshold = 40;
    int startInd=0;
    int endInd = 31;
    if(argc != 3){
      cout << "usage: ./orbTest <startind> <endInd>"<<endl;
      return -1;
    }
    startInd = atoi(argv[1]);
    endInd = atoi(argv[2]);
    /*
    if (argc < 3) {
      printf("usage: ./orbTest <Image1_Path> <Image2_Path> <ransac iter> <ransac thres> <maxsuppress dist>\n");
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
    Mat im1 = cv::imread(argv[1]);
    Mat im2 = cv::imread(argv[2]);
    if ((!im1.data || !im2.data)) {
      printf("No image data \n");
      return -1;
    }
    cvtColor(im1, im1, COLOR_RGB2GRAY);
    cvtColor(im2, im2, COLOR_RGB2GRAY);
    */
    ofstream outFile;
    Mat im1, im2;
    //bad images: 9, 10,
    //bad perfomrance: 11,12, 13
    outFile.open("orb_rThres"+to_string(rThres)+"_rIter"+to_string(rIter)+"_"+argv[1]+"_"+argv[2]+".txt");
    if(outFile.is_open()) {
        outFile<<setw(3)<<"im "<<setw(12)<<"#feature 0  "<<setw(12)<<"#feature -1 "<<setw(10)<<"#inliers  "<<setw(10)<<"#outliers "<<endl;
        for(int n=startInd ; n<endInd; ++n) {
            cout <<"im: "<<n<<endl;

            for(int m=0; m<3; ++m) {
                if(m==1) {
                    im2 = imread("imgs/exposure-1/" + to_string(n) + "_matched.pgm", cv::IMREAD_GRAYSCALE);
                    im1 = imread("imgs/exposure0/" + to_string(n) + ".jpg", cv::IMREAD_GRAYSCALE);
		    if ((!im1.data || !im2.data)) {
		      printf("No image data \n");
		      continue;
		    }
		    outFile<<setw(3)<<to_string(n)+"m";

                } else if(m == 2) {
                    im2 = imread("imgs/exposure-1/" + to_string(n) + "_eq.jpg", cv::IMREAD_GRAYSCALE);
                    im1 = imread("imgs/exposure0/" + to_string(n) + "_eq.jpg", cv::IMREAD_GRAYSCALE);
                    if ((!im1.data || !im2.data)) {
		      printf("No image data \n");
		      continue;
		    }
		    outFile<<setw(3)<<to_string(n)+"eq";
                } else {
                    im2 = imread("imgs/exposure-1/" + to_string(n) + ".jpg", cv::IMREAD_GRAYSCALE);
                    im1 = imread("imgs/exposure0/" + to_string(n) + ".jpg", cv::IMREAD_GRAYSCALE);
                    if ((!im1.data || !im2.data)) {
		      printf("No image data \n");
		      continue;
		    }
		    outFile<<setw(3)<<to_string(n)+"o";
                }
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

                Mat im1_rot, im2_rot;
                transpose(im1_un, im1_rot);
                flip(im1_rot, im1_rot, 0);
                transpose(im2_un, im2_rot);
                flip(im2_rot, im2_rot, 0);

                ORBMatching ob;
                ob.distance_threshold = distance_threshold;

                Mat des1, des2, des1_temp, des2_temp;
                vector<KeyPoint> keyp1, keyp2, keyp1_temp, keyp2_temp;
                ob.findFeatures(im1_rot, des1_temp, keyp1_temp);
                ob.findFeatures(im2_rot, des2_temp, keyp2_temp);
    
                ob.maxSuppresss(keyp1_temp, des1_temp, keyp1, des1, distThres);
                ob.maxSuppresss(keyp2_temp, des2_temp, keyp2, des2, distThres);

                //BF Matches
                vector<DMatch> matches, inlierMatches;
                ob.matchFeatures(des1, des2, keyp1, keyp2, matches);
                //ob.drawORBmatches(im1_rot, im2_rot, keyp1, keyp2, matches, "BF matches");
		if (matches.size() <= 5){
		  cout <<"matches less than 6, no ransac"<<endl;
		  outFile<<setw(12)<<keyp1.size()<<setw(12)<<keyp2.size()
			<<setw(10)<< -1<<setw(10)<< matches.size()<<endl;
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
		  //ob.drawORBmatches(im1_rot, im2_rot, keyp1, keyp2, inlierMatches, "inlier matches"+to_string(m));
		  outFile<<setw(12)<<keyp1.size()<<setw(12)<<keyp2.size()
			<<setw(10)<< inlierMatches.size()<<setw(10)<< matches.size()-inlierMatches.size()<<endl;
		}
		im1.release();
                im2.release();
            }
        }
        outFile.close();
    }
    return 0;
}
