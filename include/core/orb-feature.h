#ifndef EYEMARS_ORB_FEATURE_H_
#define EYEMARS_ORB_FEATURE_H_

#include "core/binary-feature.h"

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class OrbFeature
 *  @brief The OrbFeature class
 *  A class to store orb features
 *  which extends BinaryFeature
 */
class OrbFeature : public BinaryFeature {
 public:
  OrbFeature()
      : BinaryFeature(ORB_SIZE) {
  }
  OrbFeature(const OrbFeature& feature)
      : BinaryFeature(ORB_SIZE, feature.x(), feature.y(),
                      feature.minEigvalue(), feature.maxEigvalue(),
                      0x0) {
    descriptor_ = feature.getDescriptorCopy();
  }
  OrbFeature(float _x, float _y, float _min_e = 0.0f, float _max_e = 0.0f,
             unsigned char * _patch = NULL)
      : BinaryFeature(ORB_SIZE, _x, _y, _min_e, _max_e, _patch) {
  }
  /// size of orb descriptor
  static const int ORB_SIZE = 32;
  /// Setters
  void setDescriptor(BinaryVector* _descriptor);
};

} /** End of namespace: EyeMARS */

#endif  /// EYEMARS_ORB_FEATURE_H_
