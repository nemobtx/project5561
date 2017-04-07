#ifndef JNI_INCLUDE_BRISK_H_
#define JNI_INCLUDE_BRISK_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>

#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Common.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/VLImage.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/BriskVTFeature.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Utils.h"


class VLImage;

class BRISK {
 public:
  BRISK(int FAST_threshold = 30, int octaves = 3,
        float patternScale = 1.0f);
  BRISK(VLImage& img, int FAST_threshold = 30, int octaves = 3,
        float patternScale = 1.0f);

  ~BRISK();

  void process(VLImage& img);

 private:
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> descriptor;
};


#endif  // JNI_INCLUDE_BRISK_H_
