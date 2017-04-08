//
// Created by tong on 4/7/17.
//

#include <cmath>
#include <opencv2/opencv.hpp>
#include "HistMatching.h"

using namespace std;
using namespace cv;

void
HistMatching::histMatching(const float *src, const float *ref, HistMatching::uchar *mapping) {
  int sum_s = 0, sum_r = 0;
  uchar j = 0;
  for (int i = 0; i < 256; ++i) {
    sum_s += src[i];
    while (sum_r < sum_s) {
      sum_r += ref[j];
      ++j;
    }
    mapping[i] = abs(sum_r - sum_s) < abs(sum_r - ref[j - 1] - sum_s) ? j : j - 1;
  }
}
void HistMatching::applyMatching(const cv::Mat *src, cv::Mat *dst, uchar *mapping) {
  *dst = src->clone();
  for (int i = 0; i < src->rows; ++i) {
    for (int j = 0; j < src->cols; ++j) {
      dst->at<uchar>(i, j) = mapping[dst->at<uchar>(i, j)];
    }
  }
}
