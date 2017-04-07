#ifndef JNI_INCLUDE_RANSAC_FILTERING_H_
#define JNI_INCLUDE_RANSAC_FILTERING_H_

#include <vector>
#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "vocabulary-tree/include/UMN-Vocabulary-Tree/VLImage.h"
#include "core_database/pointfeature.h"

// #include "ransac/ransac.h"
// #include "ransac/fivepointsolver.h"
// #include "ransac/pointfeature.h"

class RansacFiltering {
 public:
  RansacFiltering() {}
  
  int matchFeatures(VLImage*, VLImage*, std::vector<int>&);
  int matchFeaturesOpenCV(VLImage*, VLImage*, std::vector<int>&);
  int imageMatch(VLImage*, VLImage*);

 private:
  int bruteforceVTFeatures(std::vector<VTFeature*>&, std::vector<VTFeature*>&, std::vector<int>&);
};
#endif  // JNI_INCLUDE_RANSAC_FILTERING_H_
