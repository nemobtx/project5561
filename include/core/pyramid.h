#ifndef EYEMARS_PYRAMID_H_
#define EYEMARS_PYRAMID_H_

#include "core/matrix2d.h"  /// To use: Matrix2d
#include "imgproc/pyramiddown.h"	/// To use: pyramidDown
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class Pyramid
 * 	@brief The pyramid class.
 * 	The class that holds the constructed pyramid from the image. Every layer of 
 *  the pyramid inherits from class Matrix2d, thus it is rensponsible to deallocate 
 *  (only) the data that was allocated by itself.
 */
class Pyramid {
 public:
	/// Default Constructor
	Pyramid() : 
    pyramid_(new Matrix2d<unsigned char>[1]), layers_(0) {}
  /// Paramitrized Constructors
	Pyramid(int _layers) : 
    pyramid_(new Matrix2d<unsigned char>[_layers]), layers_(_layers) {}
  /// Paramitrized Constructor: Build pyramid
	Pyramid(Matrix2d<unsigned char> & image, int _layers = 3) : 
    pyramid_(new Matrix2d<unsigned char>[_layers]), layers_(_layers) {	  
    Construct(image.data(), image.height(), image.width(), _layers);
  }
  /// Parametrized Constructor: Build pyramid
  Pyramid(unsigned char * image, int image_height, int image_width, int _layers = 3) : 
    pyramid_(new Matrix2d<unsigned char>[_layers]), layers_(_layers) {	  
    Construct(image, image_height, image_width, _layers);
  }
  /// Copy Constructor: shallow copy
  Pyramid(Pyramid const & _pyramid) : 
    pyramid_(new Matrix2d<unsigned char>[_pyramid.layers()]), layers_(_pyramid.layers()) {
    for (int i = 0; i < layers_; i++) {
      pyramid_[i].Set(_pyramid[i].data(), _pyramid[i].height(), _pyramid[i].width());
    }
  }
	/// Destructor
	~Pyramid(){
		if (pyramid_) {
      delete [] pyramid_;
    }
  }
  /// Methods
  /// Initilize pyramid (Deallocate first)  
  inline void Init(int _layers) {
    if (pyramid_) {
      delete[] pyramid_;
    }
    pyramid_ = new Matrix2d<unsigned char> [_layers];
    layers_ = _layers;
  }
  /// Initialize and build a Pyramid
  inline void Build(Matrix2d<unsigned char> & image, int _layers = 3) {
    Build(image.data(), image.height(), image.width(), _layers);
  }
  /// Initialize and build a Pyramid
  void Build(unsigned char *image, int image_height, int image_width,
             int _layers) {
    // Check if pyramid is initialized    
    if (pyramid_) {
      // Check if reallocation is not required
      if (layers_ == _layers && pyramid_[0].height() == image_height
          && pyramid_[0].width() == image_width) {
        pyramid_[0].Set(image, image_height, image_width);
        for (int i = 1; i < _layers; i++) {
          PyramidDown(pyramid_[i - 1].data(), pyramid_[i].data(), image_height,
                      image_width);
          image_height = (image_height + 1) >> 1;
          image_width = (image_width + 1) >> 1;
        }
        // Case Reallocation is required
      } else {
        delete[] pyramid_;
        pyramid_ = new Matrix2d<unsigned char> [_layers];
        layers_ = _layers;
        Construct(image, image_height, image_width, _layers);
      }
      // Case Pyramid is uninitialized
    } else {
      pyramid_ = new Matrix2d<unsigned char> [_layers];
      layers_ = _layers;
      Construct(image, image_height, image_width, _layers);
    }
  }
  /// Copy the pyramid: deep copy
  inline void Copy(Pyramid & _pyramid) {
    if (pyramid_) {
      delete[] pyramid_;
    }
    layers_ = _pyramid.layers();
    pyramid_ = new Matrix2d<unsigned char> [layers_];
    for (int i = 0; i < layers_; i++) {
      pyramid_[i].Copy(_pyramid[i]);
    }
  }
  /// free data 
  inline void Free() {
    if (pyramid_) {
      delete[] pyramid_;
      pyramid_ = (Matrix2d<unsigned char> *) NULL;
      layers_ = 0;
    }
  }
  /// Setters and getters
  /// Get a constructed layer
  inline Matrix2d<unsigned char> &operator[](int _layer) const {
    return pyramid_[_layer];
  }
  /// Get a constructed layer
  inline Matrix2d<unsigned char> & at(int _layer) const {
    return pyramid_[_layer];
  }
  /// Get a pointer to data the first pyramid layer (image)
  inline unsigned char * data() const {
    return pyramid_[0].data();
  }
  /// Get a pointer to data of a pyramid layer
  inline unsigned char * data(int _layer) const {
    return pyramid_[_layer].data();
  }
  /// Get the pyramid layers
  inline int layers() const {
    return layers_;
  }

 private:
  /// Build pyramid given pyramid array is allocated and layers are not allocated
  void Construct(unsigned char *image, int image_height, int image_width,
                 int _layers) {
    pyramid_[0].Set(image, image_height, image_width);
    int h, w;
    for (int i = 1; i < _layers; i++) {
      h = image_height;
      w = image_width;
      image_height = (image_height + 1) >> 1;
      image_width = (image_width + 1) >> 1;
      pyramid_[i].Resize(image_height, image_width);
      PyramidDown(pyramid_[i - 1].data(), pyramid_[i].data(), h, w);
    }
  }
  /// Data Members
	Matrix2d<unsigned char> * pyramid_;	/// pyramid: pyramid data (constructed layers)
	int layers_;         	              /// layers: number of layers of the pyramid
}; /** End of class: Pyramid */

} /** End of namespace: EyeMARS */

#endif  /// EYEMARS_PYRAMID_H_
