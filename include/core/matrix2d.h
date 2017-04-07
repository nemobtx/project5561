#ifndef EYEMARS_MATRIX2D_H_
#define EYEMARS_MATRIX2D_H_ 

#include "core/storage.h"  /// To use: Storage
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class Matrix2d
 *  @brief The Matrix class.
 *  The Matrix2d class to store Matrices. The class inherits from Storage, and thus
 *  it might or might not own its data. It is rensponsible to deallocate (only) 
 *  the data that was allocated by itself. 
 */
template<class T>
class Matrix2d : public Storage<T> {
 public:
  /// Default Constructor
  Matrix2d() : 
    Storage<T>(), height_(0), width_(0) {}
  /// Parametrized Constructor
  Matrix2d(int _height, int _width) : 
    Storage<T>(_height * _width), height_(_height), width_(_width)  {}
  /// Parametrized Constructor: shallow copy
  Matrix2d(int _height, int _width, T* _data) : 
    Storage<T>(_data), height_(_height), width_(_width) {}
  /// (Shallow) Copy Constructor
  Matrix2d(Matrix2d const & matrix) : 
    Storage<T>(matrix), height_(matrix.height()), width_(matrix.width()) {}
  /// Desctructor
  ~Matrix2d() {}
  /// Resize by allocating the right memory
  inline void Resize(int _height, int _width) {
    this->Del();
    this->Init(_height * _width);
    height_ = _height;
    width_ = _width;
  }
  /// Set the data of the Matrix without copying the data
  inline void Set(T* _data, int _height, int _width) {
    this->Del();
    this->Init(_data);
    height_ = _height;
    width_ = _width;
  }
  /// Deep copy of Matrix
  inline void Copy(Matrix2d & matrix) {
    this->Copy(matrix.data(), matrix.height(), matrix.width());
  }
  /// Deep copy of data to the data of the Matrix
  void Copy(T* _data, int _height, int _width) {
    if (this->data() && _height == height_ && _width == width_) {
      memcpy(this->data(), _data, sizeof(T) * _height * _width);
    } else {
      this->Resize(_height, _width);
      memcpy(this->data(), _data, sizeof(T) * _height * _width);
    }
  }
  /// Free data if applies (part of the class interface)
  inline void Free() {
    this->Del();
    this->Init((T*)NULL);
    height_ = 0;
    width_ = 0;
  }
  /// Setters and getters
  /// Get width
  inline int width() const {
    return width_;
  }
  /// Get hight
  inline int height() const {
    return height_;
  }

 protected:
  /// Data Members
  int height_; /**< @var height_, the Matrix height. */
  int width_; /**< @var width_, the Matrix width. */
}; /** End of class: Matrix2d */

} /** End of namespace: EyeMARS */

#endif  /// EYEMARS_MATRIX2D_H_
