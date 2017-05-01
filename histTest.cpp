//
// Created by tong on 4/7/17.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "HistMatching.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
  Mat img1, img2;
  Mat img_matched;

  int n = 1;
  if (argc > 1) {
    n = atoi(argv[1]);
  };
  for (int i = 1; i <= n; ++i) {

    img1 = imread("../imgs/exposure0/" + to_string(i) + ".jpg", IMREAD_GRAYSCALE);
    img2 = imread("../imgs/exposure-1/" + to_string(i) + ".jpg", IMREAD_GRAYSCALE);

    int histSize = 256;
    float range[] = {0, 256};
    const float *histRange = {range};
    Mat hist1, hist2;

    calcHist(&img1, 1, 0, Mat(), hist1, 1, &histSize, &histRange, true, false);
    calcHist(&img2, 1, 0, Mat(), hist2, 1, &histSize, &histRange, true, false);

    unsigned char mapping[256];
/*  for (int i = 0; i < 256; ++i) {
    mapping[i]=i;
  }*/
    HistMatching::histMatching((float *) hist2.data, (float *) hist1.data, mapping);
/*  cout << "Matching Error: "
       << HistMatching::matchingError((float *) hist2.data, (float *) hist1.data, mapping)
       << endl;*/
    HistMatching::applyMatching(&img2, &img_matched, mapping);
    imwrite("../imgs/exposure-1/" + to_string(i) + "_matched.pgm", img_matched);
  }
  imshow("target", img2);
  imshow("reference", img1);
  imshow("matched", img_matched);
  waitKey();
  return 0;
}
