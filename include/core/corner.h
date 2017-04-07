#ifndef EYEMARS_CORNER_H_
#define EYEMARS_CORNER_H_

#include <cstring>  /// To use: NULL

#include <vector>   /// To use: vector

#include "core/patch.h"   /// To use: Patch

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class Corner
 *  @brief The Corner class.
 *  A class to store corners. The class is compatible with pointers and
 *  EyeMARS datatypes for initializing the corner patch. The class is templated
 *  on the type of patch.
 */
template<class T>
class Corner {
 public:
  /// Default Constructor
  Corner () :
    x_(0), y_(0), min_eig_(0), max_eig_(0), patchset_(1) {}
  /// Parametrized Constructor
  Corner(float _x, float _y, float _min_eig = 0, float _max_eig = 0) :
    x_(_x), y_(_y), min_eig_(_min_eig), max_eig_(_max_eig), patchset_(1) {}
  /// Parametrized Constructor: Shallow initialization of the patch
  Corner(float _x, float _y, float _min_eig, float _max_eig, Patch<T> & _patch) :
    x_(_x), y_(_y), min_eig_(_min_eig), max_eig_(_max_eig), patchset_(1) {
    patchset_[0].Set(_patch.data(), _patch.height(), _patch.width());
  }
  /// Parametrized Constructor: Shallow initialization of the patch with a pointer
  Corner(float _x, float _y, float _min_eig, float _max_eig, T * _patch) :
    x_(_x), y_(_y), min_eig_(_min_eig), max_eig_(_max_eig), patchset_(1) {
    patchset_[0].Set(_patch, -1, -1);     // Patch was set with just a pointer
  }                                       // The dims should be known by the user.
  /// Parametrized Constructor: Shallow initialization of the patchset
  Corner(float _x, float _y, float _min_eig,
         float _max_eig, std::vector<Patch<T>> & _patchset) :
    x_(_x), y_(_y), min_eig_(_min_eig), max_eig_(_max_eig), patchset_(_patchset) {}
  /// Parametrized Constructor: Resize patchset and initialize patches to NULL
  Corner(unsigned int _patches_num) :
    x_(0), y_(0), min_eig_(0), max_eig_(0), patchset_(_patches_num) {}
  /// Parametrized Constructor: Allocate memory for patchset
  Corner(unsigned int _patches_num, int _patch_h, int _patch_w) :
    x_(0), y_(0), min_eig_(0), max_eig_(0), patchset_(_patches_num) {
    for (unsigned int i = 0; i < _patches_num; i++) {
      patchset_[i].Resize(_patch_h, _patch_w);
    }
  }
  /// (Shallow) Copy Constructor
  Corner(Corner const & corner) :
    x_(corner.x()), y_(corner.y()), min_eig_(corner.minEigvalue()),
    max_eig_(corner.maxEigvalue()), patchset_(corner.patchset_) {}
  /// Destructor
  virtual ~Corner() {
    patchset_.clear();
  }

  /// Methods
  /// Resize patch: Deep initialization
  inline void ResizePatch(int _patch_h, int _patch_w) {
    patchset_.at(0).Resize(_patch_h, _patch_w);
  }
  /// Resize patch by layer: Deep initialization
  inline void ResizePatch(unsigned int _index, int _patch_h, int _patch_w) {
    patchset_.at(_index).Resize(_patch_h, _patch_w);
  }
  /// Resize patchset
  inline void ResizePatchset(unsigned int _size) {
    patchset_.resize(_size);
  }
  /// Initialize patchset: Clears previous patches
  inline void InitializePatchset(unsigned int _size) {
    patchset_.clear();
    patchset_.resize(_size);
  }
  /// Initialize patchset: Deep initialation (clears previous patches)
  inline void InitializePatchset(unsigned int _size, int _patch_h, int _patch_w) {
    patchset_.resize(_size);
    for (unsigned int i = 0; i < _size; i++) {
      patchset_[i].Resize(_patch_h, _patch_w);
    }
  }
  /// Copy patch: Deep copy
  inline void CopyPatch(Patch<T> & _patch) {
    patchset_.at(0).Copy(_patch);
  }
  /// Copy patch by layer: Deep copy
  inline void CopyPatch(Patch<T> & _patch, unsigned int _index) {
    patchset_.at(_index).Copy(_patch);
  }
  /// Copy patchset: Deep copy
  inline void CopyPatchset(std::vector<Patch<T>> & _patchset) {
    patchset_.resize(_patchset.size());
    for (unsigned int i = 0; i < _patchset.size(); i++) {
      patchset_[i].Copy(_patchset[i]);
    }
  }
  /// Free allocated memory
  inline void FreePatchset() {
    patchset_.resize(1);
    patchset_[0].Free();    // patch[0].data() = NULL; height, width = 0;
  }

  /// Setters
  inline void setCoordinates(float _x, float _y) {
    x_ = _x;
    y_ = _y;
  }
  inline void setEigvalues(float _min_eig, float _max_eig) {
    min_eig_ = _min_eig;
    max_eig_ = _max_eig;
  }
  inline void setMinEigvalue(float _min_eig) {
    min_eig_ = _min_eig;
  }
  inline void setMaxEigvalue(float _max_eig) {
    max_eig_ = _max_eig;
  }
  /// Set patch: Shallow copy
  inline void setPatch(Patch<T> & _patch) {
    patchset_.at(0).Set(_patch.data(), _patch.height(), _patch.width());
  }
  /// Set patch: Shallow copy with a pointer
  inline void setPatch(T * _patch) {
    patchset_.at(0).Set(_patch, -1, -1); // Patch is set with just a pointer
  }                                      // The dims should be known by the user.
  /// Set patch by layer: Shallow copy
  inline void setPatch(Patch<T> & _patch, unsigned int _index) {
    patchset_.at(_index).Set(_patch.data(), _patch.height(), _patch.width());
  }
  /// Set patch by layer: Shallow copy by pointer
  inline void setPatch(T * _patch, unsigned int _index) {
    patchset_.at(_index).Set(_patch, -1, -1); // Patch is set with just a pointer
  }                                           // The dims should be known by the user.
  /// Set patchset: Shallow copy
  inline void setPatchset(std::vector<Patch<T>> & _patchset) {
    patchset_ = _patchset;
  }
  /// Getters
  inline float x() const {
    return x_;
  }
  inline float y() const {
    return y_;
  }
  inline float minEigvalue() const {
    return min_eig_;
  }
  inline float maxEigvalue() const {
    return max_eig_;
  }
  inline Patch<T> & patch() {
    return patchset_.at(0);
  }
  inline Patch<T> & patch(unsigned int _index) {
    return patchset_.at(_index);
  }
  inline T * patchData() {
    return patchset_.at(0).data();
  }
  inline T * patchData(unsigned int _index) {
    return patchset_.at(_index).data();
  }
  inline std::vector<Patch<T>> & patchset() {
    return patchset_;
  }
  float angle() const {
	  return angle_;
  }
  void setAngle(const float _angle) {
    angle_ = _angle;
  }
  int level() const {
    return level_;
  }
  void setLevel(int level) {
    level_ = level;
  }
 protected:
  /// Data members
  float x_;   /**<@var x_, the column coordinate of the corner in a float. **/
  float y_;   /**<@var y_, the row coordinate of the corner in a float. **/
  float min_eig_;    /**<@var min_eig_, the minimimum eigenvalue of the I matrix.**/
  float max_eig_;    /**<@var max_eig_, the maximum eigenvalue of the I matrix. **/
  int level_; /**<@var level_, level scale of corner. **/
  /**<@var patch_, the set of patches surrounding the corner in all layers.**/
  std::vector<Patch<T>> patchset_;
  float angle_;  /**<@var angle_ angle of the corner */

}; ////** End of class: Corner */

}  /** End of namespace: EyeMARS */

#endif  /// EYEMARS_CORNER_H_
