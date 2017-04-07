#ifndef JNI_INCLUDE_BINARYVTFeature_H_
#define JNI_INCLUDE_BINARYVTFeature_H_

#include <limits>
#include <ostream>
#include <iostream>

#include <Eigen/Core>

#include <stdlib.h>

#include "vocab_tree/VTFeature.h"

#include "core/binary-vector.h"
#include "core/binary-feature.h"

namespace EyeMARS {

/** @class BinaryVTFeature
 *  @brief The BinaryVTFeature class
 *  A class to store binary features 
 *  which extends BinaryFeature and VTFeature
 */
class BinaryVTFeature : public VTFeature, public BinaryFeature {
 public:
  BinaryVTFeature() {}
  BinaryVTFeature(BinaryFeature& feat) :
          VTFeature(BINARY), BinaryFeature(feat) {}
  BinaryVTFeature(int bytes, float x, float y, BinaryVector* descriptor) :
          VTFeature(BINARY) {
            setDescriptor(bytes, descriptor);
            setCoordinates(x, y);
          }
  virtual VTFeature* ComputeMean(VTFeature** feat_arr, int size);
  BinaryVTFeature* ComputeMean(BinaryVTFeature** feat_arr, int size);
  virtual float distance(VTFeature* feat);
  float distance(BinaryVTFeature* feat);
  virtual void print();
  virtual VTFeature* clone();
  virtual VTFeature* GenerateRandomVTFeature();

};

}  // End of namespace eyemars

#endif  // JNI_INCLUDE_BINARYVTFeature_H_
