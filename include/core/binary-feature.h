#ifndef EYEMARS_BINARY_FEATURE_H_
#define EYEMARS_BINARY_FEATURE_H_

#include "core/corner.h"
#include "core/binary-vector.h"

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class BinaryFeature
 *  @brief The BinaryFeature class
 *  A class to store binary features 
 *  which extends Corner
 */
class BinaryFeature: public Corner<unsigned char> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Constructors
  BinaryFeature();
  BinaryFeature(BinaryFeature &);
  BinaryFeature(int _size);
  BinaryFeature(int _size, float _x, float _y, 
                float _min_e, float _max_e, 
                unsigned char * _patch);
  BinaryFeature(int _size, float _x, float _y, 
                float _min_e, float _max_e,
                Patch<unsigned char> & _patch);
  /// Setters and Getters
  void setDescriptorCopy(int _size, const BinaryVector & _descriptor);
  void setDescriptor(int _size, BinaryVector * _descriptor);
  int getSize();
  BinaryVector * getDescriptor();
  BinaryVector getDescriptorCopy() const;
 protected:
  BinaryVector descriptor_;
  int size_;
};

}  /** End of namespace: EyeMARS */

#endif  // EYEMARS_BINARY_FEATURE_H_
