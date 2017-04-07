#ifndef JNI_INCLUDE_ORB_H_
#define JNI_INCLUDE_ORB_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>

#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Common.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/VLImage.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/OrbVTFeature.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Utils.h"

#include "core_database/pointfeature.h"

class VLImage;

class ORB {
 public:
  ORB(int o_nfeatures = 500,
         float scaleFactor = 1.2f,
         int nlevels = 8,
         int edgeThreshold = 11,
         int firstLevel = 0,
         int WTA_K = 2,
         int patchSize = 11);
  ORB(VLImage& img,
         int o_nfeatures = 500,
         float scaleFactor = 1.2f,
         int nlevels = 8,
         int edgeThreshold = 11,
         int firstLevel = 0,
         int WTA_K = 2,
         int patchSize = 11);

  ~ORB();

  void process(VLImage& img);
  void process(VLImage&, std::vector<PointFeature*>& );

  void convertFeatures(std::vector<PointFeature*>&, std::vector<cv::KeyPoint>&);
  void convertFeatures(std::vector<cv::KeyPoint>&, std::vector<PointFeature*>&);

 private:
  cv::ORB orb;
  cv::Ptr<cv::OrbFeatureDetector> ORB_detector;
  cv::Ptr<cv::DescriptorExtractor> ORB_descriptor;
};

#endif  // JNI_INCLUDE_ORB_H_
