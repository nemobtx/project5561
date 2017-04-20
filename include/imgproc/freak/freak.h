// Header for FREAK related functions
#ifndef __FREAK_H__
#define __FREAK_H__

#include "core/keypoint.h"
#include "core/freak-feature.h"
#include "opencv2/core.hpp"
#include <memory>
//#include <demo-parameters/parameters.h>

#define USE_CV_FREAK

#ifdef USE_CV_FREAK
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/core/core.hpp>
#endif

namespace EyeMARS {
typedef std::vector<std::shared_ptr<FreakFeature>> FREAKPtrVector;

class FREAKExtractor {
 public:
  FREAKExtractor();
  FREAKExtractor(int num_features);
  virtual ~FREAKExtractor();
  FREAKPtrVector FindAndExtractFreakFeaturesShort(const std::string& pyramid_folder, int img_index);
  FREAKPtrVector FindAndExtractFreakFeatures(const std::string& pyramid_folder, int img_index);
  void FindAndExtractFreakFeatures(const cv::Mat& _image, std::vector<cv::KeyPoint> & _cv_keypoints, cv::Mat & _cv_descriptor);
  FREAKPtrVector FindAndExtractFreakFeatures(const cv::Mat& input_image);
  FREAKPtrVector ExtractFreakFeatures(const std::vector<Keypoint> keypoints);

 private:
  int num_features_;
#ifdef USE_CV_FREAK
  cv::FREAK freak_extractor;
  cv::SIFT dog_detector;
#endif
};
}  // namespace EyeMARS
#endif // __FREAK_H__
