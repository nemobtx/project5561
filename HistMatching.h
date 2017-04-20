//
// Created by tong on 4/7/17.
//

#ifndef PROJECT5561_HISTMATCHING_H
#define PROJECT5561_HISTMATCHING_H

#include <opencv2/opencv.hpp>

class HistMatching {
 private:
  typedef unsigned char uchar;
 public:
  enum CDF {
    CDF_CLOSER, CDF_SMALLER, CDF_LARGER
  };
  static void histMatching(const float *src, const float *ref, uchar *mapping, CDF cdf);
  static void applyMatching(const cv::Mat *src, cv::Mat *dst, uchar *mapping);
  static float matchingError(const float *src, const float *ref, const uchar *mapping, int im_size);
};


#endif //PROJECT5561_HISTMATCHING_H
