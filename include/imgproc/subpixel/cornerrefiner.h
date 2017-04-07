#ifndef EYEMARS_CORNER_REFINER_H_
#define EYEMARS_CORNER_REFINER_H_

#include <vector>   /// To use: vector

#ifndef __ARM_NEON__
#include <opencv2/imgproc/imgproc.hpp>       /// To use: cornerSubPix()
#endif

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

/** @class CornerRefiner
 *  @brief The CornerRefiner class.
 *  The CornerRefiner class refines corners on an image as an output of a
 *  corner feature detection algorithm. THe patch size is fixed based on NEON code
 */
class CornerRefiner {
 public:
  /// Parametrized Constructor
  CornerRefiner(int _max_iterations = 8, float _epsilon = 0.1, int _window_size = 7);
  /// Destructor
  ~CornerRefiner();

  /// Main operation
  void Refine(Image & _image, std::vector<Corner<short>> * _corners);
  void Refine(const cv::Mat & src, std::vector<cv::KeyPoint> & _corners);

 private:
  /// Data Members
  int iterations_;
  float epsilon_;
  int window_size_;
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
  void ExtractPatch(float * _weights, unsigned char * _src,
                    float * _dst, int _img_width);
  void ExtractPatch_window_7(float * _weights, unsigned char * _src,
                             float * _dst, int _img_width);

  void ProcessGradientNEON(const float* subpix_buf, const float* mask, float * _a,
                           float * _b, float * _c, float * _bb1, float * _bb2);

  void ProcessGradientNEON_Window_7(const float* subpix_buf, const float* mask, float * _a,
                                    float * _b, float * _c, float * _bb1, float * _bb2);
#endif


};
/** End of class: CornerRefiner */

} /** End of namespace: EyeMARS */
#endif
