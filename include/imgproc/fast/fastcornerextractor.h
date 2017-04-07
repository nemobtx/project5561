#ifndef EYEMARS_FAST_CORNER_EXTRACTOR_H_
#define EYEMARS_FAST_CORNER_EXTRACTOR_H_
#include <vector>   /// To use: vector

#include <core/image.h>     /// To use: Image
#include <core/matrix2d.h>  /// To use: Matrix2d
#include <core/corner.h>    /// To use: Corner

#include <opencv2/core/mat.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class FastCornerExtractor
 *  @brief The FastCornerExtractor class.
 *  The Matrix2d class determines strong corners on an image,
 *  using the Fast detector.
 */
class FASTCornerExtractor {
 public:
  /// Constructors
  FASTCornerExtractor();
  FASTCornerExtractor(int _threshold, bool _nonmaxSuppression,
                      int _max_corners, int _min_dist,
                      bool _sub_pixel = false, int _iterations = 8,
                      float _epsilon = 0.1, int _window_size = 7,
                      const std::vector<std::vector<float>> & _weights_per_tile = {});
  /// Destructor
  ~FASTCornerExtractor();
  /// Main operation of FAST corner extraction
  void ExtractCorners(Image & _image, std::vector<Corner<short>> * corners);
  /// Set the weights_per_tile
  void set_weights_per_tile(const std::vector<std::vector<float>> & _weights_per_tile) {
    weights_per_tile_ = _weights_per_tile;
  }
 private:
  int threshold_;
  bool nonmaxSuppression_;
  int max_corners_;
  int min_dis_;
  bool sub_pixel_;
  int iterations_;
  float epsilon_;
  int window_size_;
  std::vector<std::vector<float>> weights_per_tile_;
  void MinDistanceSuppression(cv::Mat & _img,
                              std::vector<cv::KeyPoint> & keypoints,
                              std::vector<Corner<short>> * corners);
  void cornerSubPixel(cv::InputArray _image,
                      std::vector<Corner<short>> * _corners,
                      cv::TermCriteria criteria);
  void RefineCorners(cv::Mat & _img, std::vector<Corner<short>> * corners);
  void getRectSubPixel(cv::InputArray _image, cv::Size patchSize, cv::Point2f center,
                       cv::OutputArray _patch, int patchType);

  template<typename _Tp, typename _DTp, typename _WTp, class ScaleOp, class CastOp>
  void getRectSubPix_Cn_(const _Tp* src, size_t src_step, cv::Size src_size,
                         _DTp* dst, size_t dst_step, cv::Size win_size,
                         cv::Point2f center, int cn );
  const uchar* adjustRect( const uchar* src, size_t src_step, int pix_size,
                            cv::Size src_size, cv::Size win_size,
                              cv::Point ip, cv::Rect* pRect );
  void getRectSubPix_8u32f(const uchar* src, size_t src_step,
                            cv::Size src_size,float* dst, size_t dst_step,
                            cv::Size win_size, cv::Point2f center0, int cn);
#ifdef __ARM_NEON__
  void NEONExtractCorner(const cv::Mat & _img,
                         std::vector<cv::KeyPoint> & keypoints);
  int NEONCornerScore(const unsigned char * ptr, const int pixel[]);
  void MakeOffsets(int pixel[25], int rowStride, int patternSiz);
  void ExtractPatch(float * _weights, unsigned char * _src,
                    float * _dst, int _img_width);

  void ExtractPatch_window_7(float * _weights, unsigned char * _src,
                    float * _dst, int _img_width);

  void ProcessGradientNEON(const float* subpix_buf, const float* mask, float * _a,
                           float * _b, float * _c, float * _bb1, float * _bb2);
  void ProcessGradientNEON_Window_7(const float* subpix_buf, const float* mask, float * _a,
                           float * _b, float * _c, float * _bb1, float * _bb2);

#endif
};  /** End of class: FASTCornerExtractor */

} /** End of namespace: EyeMARS */
#endif /// EYEMARS_FAST_CORNER_EXTRACTOR_H_
