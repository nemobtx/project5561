#ifndef JNI_INCLUDE_ORBVTFeature_H_
#define JNI_INCLUDE_ORBVTFeature_H_

#include <Eigen/Core>
#include <stdlib.h>

#include "core/orb-feature.h"

#include "vocab_tree/VTFeature.h"
#include "vocab_tree/BinaryVTFeature.h"


/** @namespace eyemars
 * The eyemars namespace.
 */
namespace EyeMARS {

class OrbVTFeature : public BinaryVTFeature {
 public:
  static const int ORB_SIZE = 32;

  OrbVTFeature() {}
  OrbVTFeature(OrbFeature& feat) :
          BinaryVTFeature(feat) {}
  OrbVTFeature(float x, float y, BinaryVector descriptor) :
          BinaryVTFeature(ORB_SIZE, x, y, &descriptor) {}
};

}  // end of namespace: eyeMARS

#endif  // JNI_INCLUDE_ORBVTFeature_H_
