//
// Created by tong on 4/7/17.
//

#include <cmath>
#include <opencv2/opencv.hpp>
#include "HistMatching.h"

using namespace std;
using namespace cv;

void
HistMatching::histMatching(const float *src, const float *ref, uchar *mapping, CDF cdf) {
  int sum_s = 0, sum_r = ref[0];
  uchar j = 0;
  for (int i = 0; i < 256; ++i) {
    sum_s += src[i];
    while (sum_r < sum_s) {
      ++j;
      sum_r += ref[j];
    }
    switch (cdf) {
      case CDF_CLOSER:
        mapping[i] = abs(sum_r - sum_s) < abs(sum_r - ref[j - 1] - sum_s) ? j : j - 1;
        break;
      case CDF_SMALLER:
        mapping[i] = j - 1;
        break;
      case CDF_LARGER:
        mapping[i] = j;
        break;
    }
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
float HistMatching::matchingError(const float *src, const float *ref, const uchar *mapping, int im_size) {
  float matched[256] = {0.0f};
  for (int i = 0; i < 256; ++i) {
    matched[mapping[i]] += src[i];
  }
  float sum = 0.0f;
  for (int i = 0; i < 256; ++i) {
    sum += abs(ref[i] - matched[i]);
  }
  return sum / im_size;
}
