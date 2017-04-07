#ifndef JNI_INCLUDE_FREAK_H_
#define JNI_INCLUDE_FREAK_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>

#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Common.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/VLImage.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/FreakVTFeature.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Utils.h"


class VLImage;

class FREAK {
 public:
  FREAK();
  explicit FREAK(VLImage& img);

  ~FREAK();

  void process(VLImage& img);

 private:
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::FREAK> freak;
};


#endif  // JNI_INCLUDE_FREAK_H_
