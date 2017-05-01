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
float HistMatching::matchingError(const float *src, const float *ref, const uchar *mapping) {
  float matched[256] = {0.0f};
  for (int i = 0; i < 256; ++i) {
    matched[mapping[i]] += src[i];
  }
  float sum = 0.0f;
  for (int i = 0; i < 256; ++i) {
    sum += (ref[i] - matched[i]) * (ref[i] - matched[i]);
  }
  return sqrt(sum / 256);
}
void HistMatching::histMatchingGML(const float *src, const float *ref, uchar *mapping) {
  int sum_s = 0, sum_r = 0, fl1 = 0;
  uchar j = 0, l = 0;
  while (sum_r == 0) {
    sum_r += ref[l];
    ++l;
  }
  for (; l < 256; ++l) {
    sum_r += ref[l];
    while (sum_r > sum_s) {
      sum_s += src[j];
      ++j;
    }
    int fl = abs(sum_r - sum_s) <= abs(sum_s - src[j - 1] - sum_r) ? j : j - 1;
    for (int i = fl1; i <= fl; ++i) {
      mapping[i] = l;
    }
    if (j == 0)
      break;
    fl1 = fl + 1;
    sum_r += ref[l];
  }
}
void HistMatching::histMatchingDP(const float *src, const float *ref, uchar *mapping) {
  double C[256][256];
  int cdf_src[256], T[256][256];
  cdf_src[0] = (int) src[0];
  C[0][0] = ErrorMetric(cdf_src[0], (int) ref[0]);
  T[0][0] = -1;
  for (int j = 1; j < 256; ++j) {
    cdf_src[j] = cdf_src[j - 1] + (int) src[j];
    C[0][j] = ErrorMetric(cdf_src[j], (int) ref[0]);
    T[0][j] = -1;
  }
  for (int i = 1; i < 256; ++i) {
    C[i][0] = ErrorMetric(cdf_src[0], (int) ref[i]);
    for (int j = 1; j < 256; ++j) {
      double min_cost = ErrorMetric(cdf_src[j], (int) ref[i]);
      T[i][j] = -1;
      for (int jp = 0; jp <= j; ++jp) {
        double row_cost = ErrorMetric(cdf_src[j] - cdf_src[jp], (int) ref[i]);
        double cost = C[i - 1][jp] + row_cost;
        if (cost < min_cost) {
          min_cost = cost;
          T[i][j] = jp;
        }
      }
      C[i][j] = min_cost;
    }
  }
  double min_C = C[255][255];
  uchar min_i = 255;
  for (uchar i = 0; i < 255; ++i) {
    if (C[i][255] < min_C) {
      min_C = C[i][255];
      min_i = i;
    }
  }
  for (int j = 255; j >= 0; --min_i) {
    int jp = T[min_i][j];
    for (int k = jp + 1; k <= j; ++k) {
      mapping[k] = min_i;
    }
    j = jp;
  }
}
double HistMatching::ErrorMetric(double h1, double h2) {
  return (h1 - h2) * (h1 - h2);
}
void HistMatching::cdfMatchingDP(const float *src, const float *ref, uchar *mapping) {
  double C[256][256];
  int cdf_src[256], cdf_ref[256], T[256][256];
  cdf_src[0] = (int) src[0];
  cdf_ref[0] = (int) ref[0];
  C[0][0] = ErrorMetric(cdf_src[0], cdf_ref[0]);
  T[0][0] = -1;
  double cost0 = ErrorMetric(0, cdf_ref[0]);
  for (int j = 1; j < 256; ++j) {
    cdf_src[j] = cdf_src[j - 1] + (int) src[j];
    cdf_ref[j] = cdf_ref[j - 1] + (int) ref[j];
    C[0][j] = ErrorMetric(cdf_src[j], cdf_ref[0]);
    T[0][j] = -1;
  }
  for (int i = 1; i < 256; ++i) {
    double min_cost = cost0;
    int min_cost_j = -1;
    for (int j = 0; j < 256; ++j) {
      if (min_cost > C[i - 1][j]) {
        C[i][j] = C[i - 1][j] + ErrorMetric(cdf_src[j], cdf_ref[i]);
        min_cost_j = j;
        min_cost = C[i - 1][j];
      } else {
        C[i][j] = min_cost + ErrorMetric(cdf_src[j], cdf_ref[i]);
      };
      T[i][j] = min_cost_j;
    }
    cost0 += ErrorMetric(0, cdf_ref[i]);
  }
  for (int i = 255, j = 255; i >= 0 && j >= 0; --i) {
    int jp = T[i][j];
    for (int k = jp + 1; k <= j; ++k) {
      mapping[k] = i;
    }
    j = jp;
  }
}
