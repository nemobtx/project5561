/*
 * brisk-feature.h
 *
 *  Created on: May 19, 2014
 *      Author: mars
 */

#ifndef BRISK_FEATURE_H_
#define BRISK_FEATURE_H_

#include <core/binary-feature.h>

namespace EyeMARS {

class BriskFeature : public EyeMARS::BinaryFeature {
 public:
  BriskFeature()
      : BinaryFeature(64) {
  }
  virtual ~BriskFeature() {
  }
  BriskFeature(float _x, float _y, float _min_e, float _max_e,
               unsigned char * _patch)
      : BinaryFeature(64, _x, _y, _min_e, _max_e, _patch) {

  }
  BriskFeature(const BriskFeature& feature)
      : BinaryFeature(64, feature.x(), feature.y(),
                      feature.minEigvalue(), feature.maxEigvalue(),
                      0x0) {
    descriptor_ = feature.getDescriptorCopy();
  }
  /// size of BRISK descriptor
  int BRISK_SIZE = 64;
  /// Setters
  void setDescriptor(const BinaryVector& _descriptor) {
    descriptor_ = _descriptor;
  }
};
} /* namespace EyeMARS */
#endif /* BRISK_FEATURE_H_ */
