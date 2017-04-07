#ifndef JNI_INCLUDE_VTFeature_H_
#define JNI_INCLUDE_VTFeature_H_


#include <memory>

#include <measurement/linked-measurement.h>
// A VTFeatureType of NONE represents the zero VTFeature
enum VTFeatureType {NONE, BINARY, FLOAT};

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** A generic class to representing a visual VTFeature.
 *
 *  The VTFeature class provides space for holding the appropiate
 *  representation of a VTFeature; vector, array, etc. As well as
 *  providing methods for comparing and operating upon VTFeatures
 *  of the same type.
 */
class VTFeature {
 public:
  VTFeatureType type;

  // the index of the leaf this feature was classified into
  int index;

  explicit VTFeature(VTFeatureType type = NONE) : type(type), index(0) {}
  virtual ~VTFeature(){};
  // Returns the mean of the imput VTFeatures using the appropiate method
  virtual VTFeature* ComputeMean(VTFeature** feats, int) = 0;
  // Returns the distance between two VTFeatures
  virtual float distance(VTFeature* feat) = 0;
  virtual void print() = 0;
  virtual VTFeature* clone() = 0;
  virtual VTFeature* GenerateRandomVTFeature() = 0;

};

}  /** End of namespace: EyeMARS */


#endif  // JNI_INCLUDE_VTFeature_H_
