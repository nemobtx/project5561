/*
 * freak-feature.h
 *
 *  Created on: May 19, 2014
 *      Author: mars
 */

#ifndef FREAK_FEATURE_H_
#define FREAK_FEATURE_H_

#include <core/binary-feature.h>

namespace EyeMARS {

class FreakFeature : public EyeMARS::BinaryFeature {
 public:
  FreakFeature()
      : BinaryFeature(FREAK_SIZE) {
  }
  FreakFeature(const FreakFeature& feature)
      : BinaryFeature(FREAK_SIZE, feature.x(), feature.y(),
                      feature.minEigvalue(), feature.maxEigvalue(),
                      0x0) {
    descriptor_ = feature.getDescriptorCopy();
  }

  FreakFeature(float _x, float _y, float _min_e = 0.0f, float _max_e = 0.0f,
               unsigned char * _patch = NULL)
      : BinaryFeature(FREAK_SIZE, _x, _y, _min_e, _max_e, _patch) {
  }
  /// size of FREAK descriptor
  static const int FREAK_SIZE = 64;
  /// Setters
  void setDescriptor(const BinaryVector& _descriptor) {
    descriptor_ = _descriptor;
  }
};
} /* namespace EyeMARS */
#endif /* FREAK_FEATURE_H_ */
