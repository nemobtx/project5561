#ifndef JNI_INCLUDE_VLIMAGE_H_
#define JNI_INCLUDE_VLIMAGE_H_

#include <string>
#include <cfloat>
#include <vector>

#include "core/image.h"

#include "vocab_tree/VTFeature.h"
#include "vocab_tree/BinaryVTFeature.h"


#include "Eigen/Sparse"

/** @namespace eyemars
 * The eyemars namespace.
 */
namespace EyeMARS {

class VLImage;

typedef Eigen::SparseMatrix<float,Eigen::RowMajor> ImageDescriptor;

class VLImage {
  
 //////////////////////////////
 // additional declarations
 private:
  // static unsigned long int id;  // static variable id

 public:
  static unsigned long int id;  // static variable id
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VLImage() {
    image_ = new Image();
    image_num_ = ++id;
  }

  VLImage(std::string filename) : VLImage() {
    image_filename_ = filename;
    readImage(image_filename_);
  }

  // Eigen's aligned allocator seems to require these?
  VLImage& operator=(const VLImage& other) = default;
  explicit VLImage(const VLImage& other) = default;


  VLImage(unsigned char * img_data, int width, int height);
  
  virtual ~VLImage();

  void clearImage();

  /**
   ** @brief Reads a new image into the VLImage object, erasing the old one.
   **			Does not erase the old features, origin, etc., but does set
   ** 		the rows and cols fields appropriately.
   **
   ** @param name		the name of the file containing the new image.
   **/
  void readImage(const std::string& name);
  int matchFeatures(VLImage* img, std::vector<int>&);
  int imageMatch(VLImage* img);

  cv::Mat descriptors_;  // tempoary structure to do opencv matching
  std::vector<VTFeature*> features_;  // Detected features in the image
  ImageDescriptor quantizedImage_;

  // The image itself. Can be empty if it's not needed ( cv::Mat() )
  Image* image_;

  // The number of the image for refrence purposes
  int image_num_;
  /// The filename of the image file.
  std::string image_filename_;
};

}  // End of namespace: eyemars */

#endif  // JNI_INCLUDE_VLIMAGE_H_
