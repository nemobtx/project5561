#ifndef JNI_INCLUDE_BRISKVTFeature_H_
#define JNI_INCLUDE_BRISKVTFeature_H_

#include <Eigen/Core>
#include <stdlib.h>

#include <Eigen/Core>
#include <stdlib.h>

#include "core/brisk-feature.h"

#include "vocab_tree/VTFeature.h"
#include "vocab_tree/BinaryVTFeature.h"

namespace EyeMARS {

class BriskVTFeature : public BinaryVTFeature {
 public:
  static const int BRISK_SIZE = 64;

  BriskVTFeature() {
  }
  BriskVTFeature(BriskFeature& feat) :
          BinaryVTFeature(feat) {}
  BriskVTFeature(float x, float y, BinaryVector descriptor)
      : BinaryVTFeature(BRISK_SIZE, x, y, &descriptor) {
  }
};
}

#endif  // JNI_INCLUDE_BRISKVTFeature_H_
