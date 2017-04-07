#ifndef EYEMARS_KLT_TRACKER_H_
#define EYEMARS_KLT_TRACKER_H_

#include <vector>         // To use: std::vector

#include "core/corner.h"  // To use: Corner
#include "core/image.h"   // To use: Image
#include "core/pyramid.h" // To use:Pyramid

namespace EyeMARS {

/** @class OpticalFlowTracker
 *  @brief The OpticalFlowTracker class.
 *
 *  The Corner tracking class,
 *  it implements the Lucas Kanade feature tracking algorithm.
 */
class OpticalFlowTracker {
 public:
  // Default constructor
  OpticalFlowTracker();
  // Parameterized constructor
  OpticalFlowTracker(unsigned int _max_layers,
             unsigned int _iterations_num,
             float _min_eigen_threshold, float _epsilon,
             bool _use_initial_flow = false,
             unsigned int _window_height = 13, unsigned int _window_width = 13);
  /// Destructor
  ~OpticalFlowTracker();
  // Setters
  /// Set the number of iteration in the optimization loop
  inline void set_iterations_num(unsigned int _iterations_num) {
    iterations_num_ = _iterations_num;
  }
  /// Set the threshold of accepting a corner to be tracked
  inline void set_min_eigen_threshold(float _min_eigen_threshold) {
    min_eigen_threshold_ = _min_eigen_threshold;
  }
  /// Set the number of levels in the pyramid
  inline void set_max_levels(unsigned int _max_levels) {
    max_levels_ = _max_levels;
  }
  /// Set the optimization convergence threshold (corner displacement)
  inline void set_epsilon(float _epsilon) {
    epsilon_ = _epsilon;
  }
  /// Set the window patch dimensions
  inline void set_window_size(unsigned int _window_height,
                              unsigned int _window_width) {
    window_height_ = _window_height;
    window_width_ = _window_width;
    int extract_patch_height = _window_height + 2;
    int extract_patch_width =  _window_width + 2;

    delete[] patch1_;
    delete[] patch2_;
    delete[] gradient_;
    patch1_ = new short[extract_patch_height * extract_patch_width + 5];
    patch2_ = new short[extract_patch_height * extract_patch_width + 5];
    gradient_ = new short[(window_height_) * (window_width_) * 2 + 10];
  }

  /// Set the boolean that indicates the use of initial flow
  inline void set_use_inital_flow(bool _use_inital_flow) {
    use_inital_flow_ = _use_inital_flow;
  }
  // Getters
  /// Get the tracking status of the corners
  inline unsigned char * status() {
    return status_.data();
  }
  /// Get the pyramid constructed from image 1
  Pyramid* pyramid1() {
#ifndef __ARM_NEON__
    pyramid1_->Build(image1_, max_levels_);
#endif
    return pyramid1_;
  }
  /// Get the pyramid constructed from image 2
  Pyramid* pyramid2() {
#ifndef __ARM_NEON__
    pyramid2_->Build(image2_, max_levels_);
#endif
    return pyramid2_;
  }
  // Operation functions
  /// Track corners1 from image1 to image2
  /// Note: NEON code is called only for patch size 13x13
  void Track(Image & _image1, Image & _image2,
             std::vector<Corner<short>> & _corners1,
             std::vector<Corner<short>> & _corners2);
  /// Tracks corners1, from the previous Track call to image2.
  /// Note:
  /// - NEON code is called only for patch size 13x13
  /// - The function builds the pyramid for the passed image (image2)
  /// and reuses the constructed pyramid for image1.
  void Track(Image & _image2,
             std::vector<Corner<short>> & _corners1,
             std::vector<Corner<short>> & _corners2);
  /// Track corners1 from first image to last image in the image vector.
  /// Note: NEON code is called only for patch size 13x13
//  void GreedyTrack(std::vector<Image> & image_vector,
//                   std::vector<Corner<short>> & _corners1,
//                   std::vector<Corner<short>> & final_vector,
//                   float sufficient_percentage, int max_splits,
//                   int inital_level);

  /// Track corners using the Opencv function,
  /// for the non-ARM platforms and parameters.
  void TrackOpenCV(Image & _image1, Image & _image2,
                   std::vector<Corner<short>> & _corners1,
                   std::vector<Corner<short>> & _corners2);
  void TrackOpenCV(Image & _image1, Image & _image2,
                   std::vector<Corner<short>> & corners1,
                   std::vector<Corner<short>> & corners2,
                   std::vector<unsigned char> & _status, int level);
#ifdef __ARM_NEON__
  void TrackCorner_KeyFrame_NEON(Matrix2d<unsigned char>  & image1_,
                                 Matrix2d<unsigned char>  & image2_,
                                 int _level, int _max_levels,
                                 std::vector<Corner<short>> & _corners1,
                                 std::vector<Corner<short>> & _corners2,
                                 std::vector<unsigned char> & _status);
  /// Track the corners across the specified pyramid level.
  void TrackCorners(Matrix2d<unsigned char> & image1_,
                    Matrix2d<unsigned char> & image2_,
                    unsigned int _level,
                    std::vector<Corner<short>> & _corners1,
                    std::vector<Corner<short>> & _corners2);
#endif

 private:
  // Data members
  /// Number of levels in the pyramid
  unsigned int max_levels_;
  /// Patch window height
  unsigned int window_height_;
  /// Patch window width
  unsigned int window_width_;
  /// Number of iterations in the optimization loop
  unsigned int iterations_num_;
  /// The threshold of accepting a corner to be tracked
  float min_eigen_threshold_;
  /// Optimization convergence threshold (corner displacement)
  float epsilon_;
  /// A boolean indicates the use of initial computed flow for the tracked corners
  bool use_inital_flow_;

  /// The input image 1
  Image image1_;
  /// The input image 2
  Image image2_;
  /// The pyramid constructed from image 1
  Pyramid* pyramid1_;
  /// The pyramid constructed from image 2
  Pyramid* pyramid2_;
  /// The patch extracted from corner 1
  short * patch1_;
  /// The patch extracted from corner 2
  short * patch2_;
  /// The gradient of calculated from patch 1
  short* gradient_;
  short* initial_gradient_;
  /// The tracking status of corners 2 (1 tracked - 0 untracked)
  std::vector<unsigned char> status_;
}; // End of OpticalFlowTracker


class KltTracker {
 public:
  /// Constructor and Destructor
  KltTracker();
  /// Parametrized Constructor
  KltTracker(int _window_height, int _window_width, int _max_layers,
             int _iterations_num, float _min_eigen_threshold, float _epsilon,
             int _flags);
  /// Destructor
  ~KltTracker();
  /// Setters and Getters
  void setMaxIterations(int _iterations_num);
  void setEpsilon(float _epsilon);
  void setEigenThreshold(float _min_eigen_threshold);
  void setFlags(int _flags);
  void setWindowSize(int _window_height, int _window_width);
  void setMaxLayers(int _max_layers);
  /// Getters
  int maxLayers() const;
  int flags() const;
  unsigned char * status();
  float * minEigvalues();
  float * error();
  Pyramid * pyramid1();
  Pyramid * pyramid2();

  /// Operation functions
  /// track function: corners1 must be one of two things:
  ///                 1. An array of corners, where all corners have their patches,
  ///                    which are compatible with the pyramid
  ///                 2. An array of corners, where all corners are from the
  ///                    previous image, the patches of these corners will be filled
  ///                 corners2 should be empty vector of corners
  void track(Image & image2, std::vector<Corner<short>*> & corners1,
             std::vector<Corner<short>*> & corners2);
  /// track function: corners1 should be the output of Harris Corner Extractor
  ///                 corners2 should be an empty vector of corners
  void track(Image & image1, Image & image2,
             std::vector<Corner<short>*> & corners1,
             std::vector<Corner<short>*> & corners2);
  /// track function: Opencv based function
  void trackOpenCV(Image & image1, Image & image2,
                   std::vector<Corner<short>*> & corners1,
                   std::vector<Corner<short>*> & corners2);
 private:
  /// Data Members
  int window_height_;
  int window_width_;
  int max_layers_;
  int iterations_num_;
  float min_eigen_threshold_;
  float epsilon_;
  int flags_;

  std::vector<unsigned char> status_;
  std::vector<float> min_eigen_;
  std::vector<float> error_;

  Image prev_image_;

  Pyramid * pyramid1_;
  Pyramid * pyramid2_;
};  /** End of KltTracker */

} // End of namespace: EyeMARS
#endif  // End of file: EYEMARS_KLT_TRACKER_H_
