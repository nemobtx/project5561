#ifndef __DOG_SHARED__
#define __DOG_SHARED__

#include "core/keypoint.h"
#include "opencv2/core.hpp"
#include "assert.h"
#include <opencv2/features2d/features2d.hpp>


namespace EyeMARS {
// Set some constants
const int NUM_DOG_IMAGE = 2;  // number of DOG images EACH octave
const int NUM_OCTAVES = 2;    // number of octave
const int NUM_SCALES = NUM_DOG_IMAGE * NUM_OCTAVES;
const int MAX_SUBPIXEL_UPDATE_DISTANCE = 1;
const float SUBPIXEL_THRES_r = 10;
const float SUBPIXEL_THRES_ratio = (SUBPIXEL_THRES_r + 1)
    * (SUBPIXEL_THRES_r + 1) / SUBPIXEL_THRES_r;
const int LAPLACIAN_THRESHOLD = 2;

template<typename precision=float>
float InterpolatePixelBilinear(const cv::Mat& image, float x, float y) {
  assert(image.data);
  int xp, yp;
  int xp_plus_1, yp_plus_1;
  float w0, w1, w2, w3;
  const precision* p0;
  const precision* p1;
  float result;

  xp = (int) x;
  yp = (int) y;

  xp_plus_1 = xp+1;
  yp_plus_1 = yp+1;

  p0 = image.ptr<precision>(yp);
  assert(p0);
  p1 = image.ptr<precision>(yp_plus_1);
  assert(p1);

  w0 = (xp_plus_1 - x)  * (yp_plus_1 - y);
  w1 = (x         - xp) * (yp_plus_1 - y);
  w2 = (xp_plus_1 - x)  * (y         - yp);
  w3 = (x         - xp) * (y         - yp);

  result = w0*p0[xp] + w1*p0[xp_plus_1] + w2*p1[xp] + w3*p1[xp_plus_1];

  return result;
}
void RetainBestFeatures(int max_num_features, std::vector<Keypoint>* keypoints);
void RetainBestFeatures(int max_num_features, std::vector<cv::KeyPoint>* keypoints);
void upsample_point_bilinear(float& xp, float& yp, float x, float y,
                             int octave);
} // namespace EyeMARS

#endif // __DOG_SHARED__
