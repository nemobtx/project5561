#ifndef EYEMARS_STORAGE_H_
#define EYEMARS_STORAGE_H_

#include <cstring>  /// To use: NULL
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class Storage
 * 	@brief The Storage class.
 * 	The class to keep and maintain data. It might or might not own its data.
 *  It is rensponsible to deallocate (only) the data that was allocated by itself.
 *  This feature is essential for interfacing with other 3rd-party libraries sefely,
 *  as well as staticly allocated data, without having to keep track of any memory.
 *  allocation.
 */
template<class T>
class Storage {
 public:
	/// Default Constructor
	Storage() : 
    data_((T*)NULL), local_alloc_(false) {}
	/// Parametrized Constructor
	Storage(int _size) : 
    data_(new T [_size]), local_alloc_(true) {}
  /// Parametrized Constructor: without copying
	Storage(T* _data) : 
    data_(_data), local_alloc_(false) {}
  /// Copy constructor: shallow copy
  Storage(Storage const & storage) :
    data_ (storage.data()), local_alloc_(false){}
	/// Destructor
	virtual ~Storage() {   
    if (data_ && local_alloc_) { // to avoid deleting something not created locally
      local_alloc_ = false;      // with the new operator i.e openCV data
      delete[] data_;
    }
  }
  /// Methods
  /// Free data if applies (part of the class interface)
  virtual inline void Free() {
    Del();
    data_ = (T*) NULL;
  }
  /// Setters and getters
  /// Get a pointer to data
  inline T* data() const {
    return data_;
  }
  /// Get a reference to the pointer to data
  inline T* &dataReference() {
    return *&data_;
  }

 protected:
  /// Methods
  /// Deep initialization: new memory allocation  
  inline void Init(int _size) {
    data_ = new T[_size];
    local_alloc_ = true;
  }
  /// Shallow initialization: without copying data
  inline void Init(T* _data) {
    data_ = _data;
    local_alloc_ = false;
  }
  /// Delete data if applies
  inline void Del() {
    if (data_ && local_alloc_) {  // to avoid deleting something not created locally
      local_alloc_ = false;     // with the new operator i.e openCV data
      delete[] data_;
    }
  }
  /// Data Members
	T* data_;	    /// data_: Pointer to data
  bool local_alloc_;  /**< @var local_alloc_, indicates if the data was allocated locally. */
}; /** End of class: Storage */
 
}; /** End of namespace: EyeMARS */

#endif  /// EYEMARS_STORAGE_H_
