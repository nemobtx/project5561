#ifndef JNI_INCLUDE_SIFTVTFeature_H_
#define JNI_INCLUDE_SIFTVTFeature_H_

#include <Eigen/Core>
#include <stdlib.h>

#include "vocabulary-tree/include/UMN-Vocabulary-Tree/VTFeature.h"
#include "vocabulary-tree/include/UMN-Vocabulary-Tree/FloatVTFeature.h"



class SIFTVTFeature : public FloatVTFeature {
 public:
  static const int SIFT_SIZE = 128;

  SIFTVTFeature() {}
  SIFTVTFeature(float x, float y, FloatVector descriptor) :
          FloatVTFeature(SIFT_SIZE, x, y, descriptor) {}
};

#endif  // JNI_INCLUDE_SIFTVTFeature_H_
