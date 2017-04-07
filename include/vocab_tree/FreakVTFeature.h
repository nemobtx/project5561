#ifndef JNI_INCLUDE_FREAKVTFeature_H_
#define JNI_INCLUDE_FREAKVTFeature_H_

#include <Eigen/Core>
#include <stdlib.h>

#include <Eigen/Core>
#include <stdlib.h>

#include "core/freak-feature.h"

#include "vocab_tree/VTFeature.h"
#include "vocab_tree/BinaryVTFeature.h"

namespace EyeMARS {

class FreakVTFeature : public BinaryVTFeature {
 public:
  static const int FREAK_SIZE = 64;

  FreakVTFeature() {
  }
  FreakVTFeature(FreakFeature& feat) :
          BinaryVTFeature(feat) {}
  FreakVTFeature(float x, float y, BinaryVector descriptor)
      : BinaryVTFeature(FREAK_SIZE, x, y, &descriptor) {
  }
};
}

#endif  // JNI_INCLUDE_FREAKVTFeature_H_
