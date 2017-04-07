#ifndef EYEMARS_PATCH_H_
#define EYEMARS_PATCH_H_

#include "core/matrix2d.h" /// To use: Matrix2d
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class Patch
 * 	@brief The Patch class.
 * 	The Patch class to store patches (corner patches, gradient patches,
 *  image segments etc). The class inherits from Matrix2d. It is rensponsible 
 *  to deallocate (only) the data that was allocated by itself.
 */
template<class T>
class Patch : public Matrix2d<T> {
 public:
  /// Default Constructor
  Patch() : Matrix2d<T>() {}
  /// Parametrized Constructor
  Patch(int _height, int _width) : Matrix2d<T>(_height, _width) {}
  /// Parametrized Constructor: shallow copy
  Patch(int _height, int _width, T* _data) : Matrix2d<T>(_height, _width, _data) {}
  /// (Shallow) Copy Constructor
  Patch(Patch const & patch) : Matrix2d<T>(patch) {}
  /// Destructor
  ~Patch() {}
}; /** End of class: Patch */
 
}; /** End of namespace: EyeMARS */

// this is much better but I couldn't make it work on Android
//template <class T>
//using Patch = Matrix2d<T>;
#endif  /// EYEMARS_PATCH_H_
