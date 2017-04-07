#ifndef EYEMARS_IMAGE_H_
#define EYEMARS_IMAGE_H_ 

#include <cstdio>    /// To use: perror()
#include <string>    /// To use: string
#include <fstream>   /// To use: ifstream, ofstream
#include <opencv2/core.hpp> /// To use Mat
#include "core/matrix2d.h"  /// To use: Matrix2d
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class Image
 *  @brief The Image class.
 *  The image class to store images. The class inherits from Storage, and thus
 *  it might or might not own its data. It is rensponsible to deallocate (only) 
 *  the data that was allocated by itself.
 */
class Image : public Matrix2d<unsigned char> {
 public:
  /// Default Constructor
  Image() : 
    Matrix2d<unsigned char>() {}
  /// Parametrized Constructor
  Image(int _height, int _width) : 
    Matrix2d<unsigned char>(_height , _width) {}
  /// Parametrized Constructor: shallow copy
  Image(int _height, int _width, unsigned char* _data) : 
    Matrix2d<unsigned char>(_height, _width, _data) {}      // shallow copy
  /// Copy Constructor: shallow copy
  Image(Image const & image) : 
    Matrix2d<unsigned char>(image) {}   // deep copy
  /// Conversion from cv::Mat to Image
  Image(cv::Mat& mat) {    
    if (mat.type() == CV_8U) {
      this->Init(mat.data); // shallow copy
      height_ = mat.rows;
      width_ = mat.cols;
    } else {
      perror ("Type must be grayscale (CV_8U)");
    }
  }
  /// Desctructor
  ~Image() {}
  /// Methods
  /// Conversion from Image to cv::Mat
  inline cv::Mat CvMat() {
    return cv::Mat(height_, width_, CV_8U, data_, 0);
  }
  /// Utilites for reading form files
  void LoadPGMFile(std::string filename) {
    char junk[100];
    std::ifstream pgmfile;
    pgmfile.open(filename.c_str(), std::ios::binary);
    pgmfile.getline(junk, 100);  // P5
    int height = 0;
    int width = 0;
    pgmfile >> width >> height;  // Width, Height
    if (height != height_ || width != width_) {
      this->Resize(height, width);
    }
    pgmfile.getline(junk, 100);  // Endline
    pgmfile.getline(junk, 100);  // Max Pixel Value
    pgmfile.read(reinterpret_cast<char*>(data_), height_ * width_);
    pgmfile.close();
  }
  /// Utilites for writing to files
  void WritePGMFile(std::string filename) {
    std::ofstream pgmfile;
    pgmfile.open(filename.c_str(), std::ios_base::binary);
    pgmfile << "P5\n" << width_ << " " << height_ << "\n255\n";
    pgmfile.write(reinterpret_cast<char*>(data_), height_ * width_);
    pgmfile.close();
  }
}; /** End of class: Image */

}  /** End of namespace: EyeMARS */

#endif  /// EYEMARS_IMAGE_H_
