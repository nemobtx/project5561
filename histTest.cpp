//
// Created by tong on 4/7/17.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "HistMatching.h"

using namespace std;
using namespace cv;

int main() {
  Mat img1 = imread("imgs/d0.jpg");
  Mat img2 = imread("imgs/e-1.jpg");

  cvtColor(img1, img1, COLOR_RGB2GRAY);
  cvtColor(img2, img2, COLOR_RGB2GRAY);

  int histSize = 256;
  float range[] = {0, 256};
  const float *histRange = {range};
  Mat hist1, hist2;

  calcHist(&img1, 1, 0, Mat(), hist1, 1, &histSize, &histRange, true, false);
  calcHist(&img2, 1, 0, Mat(), hist2, 1, &histSize, &histRange, true, false);

/*  cout << hist1 << endl;
  cout << hist2 << endl;*/

  unsigned char mapping[256];
  HistMatching::histMatching((float *) hist2.data, (float *) hist1.data, mapping);
  Mat img_matched;
  HistMatching::applyMatching(&img2, &img_matched, mapping);

/*  for (int i = 0; i < 256; ++i) {
    cout << (int) mapping[i] << " ";
  }*/

  imshow("matched", img_matched);
  waitKey();
  return 0;
}
