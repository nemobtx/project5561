#ifndef EYEMARS_HARRIS_CORNER_EXTRACTOR_H_
#define EYEMARS_HARRIS_CORNER_EXTRACTOR_H_

#include <vector>   /// To use: vector

#ifndef __ARM_NEON__
#include <opencv2/imgproc/imgproc.hpp> /// To use: goodFeaturesToTrack()
#endif

#ifdef __ARM_NEON__
#include "imgproc/harris/cornerextractor.h" /// To use: cornerExtractor()
#endif

#include "core/image.h"     /// To use: Image
#include "core/matrix2d.h"  /// To use: Matrix2d
#include "core/corner.h"    /// To use: Corner

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class HarrisCornerExtractor
 *  @brief The HarrisCornerExtractor class.
 *  The Matrix2d class determines strong corners on an image,
 *  using the harris detector.
 */
class HarrisCornerExtractor {
 public:
  HarrisCornerExtractor() :
      min_distance_(10), quality_level_(0.006), max_corners_(200) {
  #ifdef __ARM_NEON__
      grid_.setCellSize(min_distance_);
  #endif
    }
  HarrisCornerExtractor(int _min_distance, float _quality_level, int _max_corners) :
    min_distance_(_min_distance), quality_level_(_quality_level),
    max_corners_ (_max_corners) {
#ifdef __ARM_NEON__
    grid_.setCellSize(min_distance_);
#endif
  }
  /// Destructor
 ~HarrisCornerExtractor() {}
  /// Setters and Getters
  inline void setMinimumDistance(int _min_distance) {
    min_distance_ = _min_distance;
  }
  inline void setQualityLevel(float _quality_level) {
    quality_level_ = _quality_level;
  }
  inline void setMaximumCorners(int _max_corners) {
    max_corners_ = _max_corners;
  }
  inline int minimumDistance() {
    return min_distance_;
  }
  inline float qualityLevel() {
    return quality_level_;
  }
  inline int maximumCorners() {
    return max_corners_;
  }
  /// Main operation of Harris corner extraction
  void extractCorners(Image & image, std::vector<Corner<short>> & corners) {
#ifndef __ARM_NEON__
    cv::Mat img = image.CvMat();
    std::vector<cv::Point2f> cnrs;
    goodFeaturesToTrack(img, cnrs, max_corners_, (double)quality_level_,
                        min_distance_, cv::Mat(), 3, false);
    corners.resize(cnrs.size());
    for (unsigned int i = 0; i < cnrs.size(); i++) {
      corners[i].setCoordinates(cnrs[i].x, cnrs[i].y);
    }
#endif

#ifdef __ARM_NEON__
grid_.Resize(image.height(), image.width());
if (gradient_.height() != (image.height() - 2) || gradient_.width() != (2 * image.width())) {
  gradient_.Resize(image.height() - 2, 2 * image.width());
  harris_values_.Resize(image.height() - 4, image.width() - 2);
  max_eigen_.Resize(image.height() - 4, image.width() - 2);
}
memset(gradient_.data(), 0, (size_t)(gradient_.height() * gradient_.width()) * sizeof(unsigned char));
memset(harris_values_.data(), 0, (size_t)(harris_values_.height() * harris_values_.width()) * sizeof(float));
memset(max_eigen_.data(), 0, (size_t)(max_eigen_.height() * max_eigen_.width()) * sizeof(float));
/// Run Harris
CornerExtractor(image.data(), gradient_.data(), harris_values_.data(),
                image.height(), image.width(), corners , &grid_,
                min_distance_, quality_level_, max_corners_, max_eigen_.data());
#endif
  }

 private:
  /// Data Members
  int min_distance_;
  float quality_level_;
  int max_corners_;
#ifdef __ARM_NEON__
  Matrix2d<unsigned char> gradient_;
  Matrix2d<float> harris_values_;
  Matrix2d<float> max_eigen_;
  Grid  grid_;
#endif
};  /** End of class: HarrisCornerExtractor */

} /** End of namespace: EyeMARS */

#endif /// EYEMARS_HARRIS_CORNER_EXTRACTOR_H_
