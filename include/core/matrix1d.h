#ifndef EYEMARS_MATRIX1D_H_
#define EYEMARS_MATRIX1D_H_

#include "core/storage.h" /// To use: Storage
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class Matrix1d
 * 	@brief The Matrix1d class.
 * 	The Matrix1d class to store Matrices. The class inherits
 *  from Storage. It is rensponsible to deallocate (only) the data that was 
 *  allocated by itself.   
 */
template<class T>
class Matrix1d : public Storage<T> {
 public:
	/// Default Constructor
	Matrix1d() : 
    Storage<T>(), size_(0) {}
	/// Parametrized Constructor
	Matrix1d(int _size) : 
    Storage<T>(_size), size_(_size) {}
  /// Parametrized Constructor: shallow copy
	Matrix1d(int _size, T* _data) : 
    Storage<T>(_data), size_(_size) {}
  /// (Shallow) Copy Constructor
  Matrix1d(Matrix1d const & matrix) : 
    Storage<T>(matrix.data()), size_(matrix.size()) {}
	/// Destructor
	~Matrix1d() {}
  /// Methods
  /// Resize by allocating the appropriate memory
  inline void Resize(int _size) {
    this->Del();
    this->Init(_size);
    size_ = _size;
  }
  /// Set the data of the matrix without copying the data
  inline void Set(T* _data, int _size) {
    this->Del();
    this->Init(_data);   // shallow copy
    size_ = _size;
  }
  /// Deep copy of the matrix
  inline void Copy(Matrix1d & matrix) {
    this->Copy(matrix.data(), matrix.size());
  }
  /// Deep copy of data to the data of the matrix
  void Copy(T* _data, int _size) {
    if (this->data() && _size == size_) {
      memcpy(this->data(), _data, sizeof(T) * _size);
    } else {
      this->Resize(_size);
      memcpy(this->data(), _data, sizeof(T) * _size);
    }
  }
  /// Free data if applies (part of the class interface)
  inline void Free() {
    this->Del();
    size_ = 0;
    this->Init((T*)NULL);
  }
  /// Setters and getters  
  /// Get size
  inline int size() const {
    return size_;
  }
 protected:
  /// Data Members
	int size_;			  /// size_: size of the Matrix1d
}; /** End of class: Matrix1d */
 
}; /** End of namespace: EyeMARS */

#endif  // EYEMARS_MATRIX1D_H_
