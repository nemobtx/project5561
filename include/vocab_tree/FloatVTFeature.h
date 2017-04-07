#ifndef JNI_INCLUDE_FLOATVTFeature_H_
#define JNI_INCLUDE_FLOATVTFeature_H_

#include <Eigen/Core>
#include <stdlib.h>

#include "vocab_tree/VTFeature.h"


/** @namespace eyemars
 * The eyemars namespace.
 */
namespace EyeMARS {

typedef Eigen::Matrix<float, 1, Eigen::Dynamic> FloatVector;

class FloatVTFeature : public VTFeature {
 public:
  int length;
  float x, y;

  FloatVector descriptor;

  FloatVTFeature() {}
  FloatVTFeature(int length, float x, float y, FloatVector descriptor) :
          VTFeature(FLOAT), length(length), x(x), y(y), descriptor(descriptor) {}
  virtual VTFeature* ComputeMean(VTFeature** feat_arr, int size);
  FloatVTFeature* ComputeMean(FloatVTFeature** feat_arr, int size);
  virtual float distance(VTFeature* feat);
  float distance(FloatVTFeature* feat);
  virtual void print();
  virtual VTFeature* clone();
  virtual VTFeature* GenerateRandomVTFeature();

};

}  // end of namespace: eyeMARS

#endif  // JNI_INCLUDE_FLOATVTFeature_H_
