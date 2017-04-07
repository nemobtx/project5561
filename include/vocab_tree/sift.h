#ifndef JNI_INCLUDE_SIFT_H_
#define JNI_INCLUDE_SIFT_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>

#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Common.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/VLImage.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/SIFTVTFeature.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/Utils.h"

class VLImage;

class SIFT {
 public:
    SIFT(int nfeatures = 0, int nOctaveLayers = 3,
         double contrastThreshold = 0.04, double edgeThreshold = 10,
         double sigma = 1.6);
    SIFT(VLImage& img, int nfeatures = 0, int nOctaveLayers = 3,
         double contrastThreshold = 0.04, double edgeThreshold = 10,
         double sigma = 1.6);

    ~SIFT();

    void process(VLImage& img);
 private:
    cv::SIFT sift;
};

#endif  // JNI_INCLUDE_SIFT_H_
