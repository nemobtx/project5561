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
  static void histMatching(const float *src, const float *ref, uchar *mapping);
  static void applyMatching(const cv::Mat *src, cv::Mat *dst, uchar *mapping);
};


#endif //PROJECT5561_HISTMATCHING_H
