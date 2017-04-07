#ifndef EYEMARS_ORB_KEYPOINT_EXTRACTOR_H_
#define EYEMARS_ORB_KEYPOINT_EXTRACTOR_H_
#include <vector>   /// To use: vector

#include <core/image.h>     /// To use: Image
#include <core/matrix2d.h>  /// To use: Matrix2d
#include <core/corner.h>    /// To use: Corner
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "OPENCV_orbARM.h"

#include "types.h"

#define USE_MARS_ORB

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {
  static const int ORB_SIZE = 32;
  const float HARRIS_K = 0.04f;
  const int HARRIS_SCORE = 0;
  const int FAST_SCORE = 1;

  /** @class ORB FEATURE CLASS
 *  @brief The ORB class.
 */
class orbextractor {
 public:
  /// Constructors
  orbextractor();
  orbextractor(int _max_features, float _scale_factor,
               int _pyramid_level, int _edge_threshold,
               int _wta_k, int _patch_size);
  /// Destructor
  ~orbextractor();
  void ExtractKeypointsDescriptors(Image & _image,
                                   std::vector<OrbFeature *> * _features);
  void ExtractKeypointsDescriptors(const cv::Mat & _image,
                                   std::vector<cv::KeyPoint> & _features,
                                   cv::Mat & _descriptors);
 private:
  void ExtractKeypointsDescriptorsImpl(cv::InputArray _image, cv::InputArray _mask,
                                       std::vector<cv::KeyPoint> & _keypoints,
                                       cv::OutputArray _descriptors,
                                       bool useProvidedKeypoints);
  void computeKeyPoints(const std::vector<cv::Mat> & _imagePyramid,
                        const std::vector<cv::Mat> & _maskPyramid,
                        std::vector<std::vector<cv::KeyPoint>> & _allKeypoints,
                        int _nfeatures, int _firstLevel, double _scaleFactor,
                        int _edgeThreshold, int _patchSize, int _scoreType);
  void computeDescriptors(const cv::Mat & _image, std::vector<cv::KeyPoint> & _keypoints,
                          cv::Mat& _descriptors, const std::vector<cv::Point> & _pattern,
                          int _dsize, int _WTA_K);
  void computeOrientation(const cv::Mat & _image, std::vector<cv::KeyPoint> & _keypoints,
                          int _halfPatchSize, const std::vector<int> & _umax);
  void initializeOrbPattern(const cv::Point * _pattern0,
                            std::vector<cv::Point> & _pattern, int _ntuples,
                            int _tupleSize, int _poolSize);
  void makeRandomPattern(int _patchSize, cv::Point* _pattern, int _npoints);
  inline float getScale(int _level, int _firstLevel, double _scaleFactor) const {
    return (float)std::pow(_scaleFactor, (double)(_level - _firstLevel));
  }
  inline int descriptorSize() const {
    return ORB_SIZE;
  }
  inline int descriptorType() const {
    return CV_8U;
  }
  inline void HarrisResponses(const cv::Mat & _img, std::vector<cv::KeyPoint> & _pts,
                            int _blockSize, float _harris_k) {
    CV_Assert(_img.type() == CV_8UC1 && _blockSize * _blockSize <= 2048);

    size_t ptidx, ptsize = _pts.size();

    const uchar* ptr00 = _img.ptr<uchar>();
    int step = (int) (_img.step / _img.elemSize1());
    int r = _blockSize / 2;

    float scale = (1 << 2) * _blockSize * 255.0f;
    scale = 1.0f / scale;
    float scale_sq_sq = scale * scale * scale * scale;

    cv::AutoBuffer<int> ofsbuf(_blockSize * _blockSize);
    int* ofs = ofsbuf;
    for (int i = 0; i < _blockSize; i++)
      for (int j = 0; j < _blockSize; j++)
        ofs[i * _blockSize + j] = (int) (i * step + j);

    for (ptidx = 0; ptidx < ptsize; ptidx++) {
      int x0 = cvRound(_pts[ptidx].pt.x - r);
      int y0 = cvRound(_pts[ptidx].pt.y - r);

      const uchar* ptr0 = ptr00 + y0 * step + x0;
      int a = 0, b = 0, c = 0;

      for (int k = 0; k < _blockSize * _blockSize; k++) {
        const uchar* ptr = ptr0 + ofs[k];
        int Ix = (ptr[1] - ptr[-1]) * 2 + (ptr[-step + 1] - ptr[-step - 1])
            + (ptr[step + 1] - ptr[step - 1]);
        int Iy = (ptr[step] - ptr[-step]) * 2 + (ptr[step - 1] - ptr[-step - 1])
            + (ptr[step + 1] - ptr[-step + 1]);
        a += Ix * Ix;
        b += Iy * Iy;
        c += Ix * Iy;
      }
      _pts[ptidx].response = ((float) a * b - (float) c * c
          - _harris_k * ((float) a + b) * ((float) a + b)) * scale_sq_sq;
    }
  }
  inline float IC_Angle(const cv::Mat & _image, const int _half_k, cv::Point2f _pt,
                        const std::vector<int> & _u_max) {
    int m_01 = 0, m_10 = 0;

    const uchar * center = &_image.at<uchar>(cvRound(_pt.y), cvRound(_pt.x));

    // Treat the center line differently, v=0
    for (int u = -_half_k; u <= _half_k; ++u)
      m_10 += u * center[u];

    // Go line by line in the circular patch
    int step = (int) _image.step1();
    for (int v = 1; v <= _half_k; ++v) {
      // Proceed over the two lines
      int v_sum = 0;
      int d = _u_max[v];
      for (int u = -d; u <= d; ++u) {
        int val_plus = center[u + v * step], val_minus = center[u - v * step];
        v_sum += (val_plus - val_minus);
        m_10 += u * (val_plus + val_minus);
      }
      m_01 += v * v_sum;
    }

    return cv::fastAtan2((float) m_01, (float) m_10);
  }
  inline void computeOrbDescriptor(const cv::KeyPoint & _kpt,
                                   const cv::Mat & _img, const cv::Point * _pattern,
                                   uchar * _desc, int _dsize, int _WTA_K) {
    float angle = _kpt.angle;
    angle *= (float) (CV_PI / 180.f);
    const uchar* center = &_img.at<uchar>(cvRound(_kpt.pt.y), cvRound(_kpt.pt.x));
    int step = (int) _img.step;
    float a = (float) cos(angle), b = (float) sin(angle);

#define GET_VALUE(idx) \
        center[cvRound(_pattern[idx].x*b + _pattern[idx].y*a)*step + \
               cvRound(_pattern[idx].x*a - _pattern[idx].y*b)]

    if (_WTA_K == 2) {
#if defined (USE_MARS_ORB) && defined (__ARM_NEON__)
      computeARM(_dsize, _pattern, _desc, a, b, step, center);
#else
      for (int i = 0; i < _dsize; ++i, _pattern += 16) {
        int t0, t1, val;
        t0 = GET_VALUE(0);
        t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2);
        t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4);
        t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6);
        t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8);
        t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10);
        t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12);
        t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14);
        t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        _desc[i] = (uchar) val;
      }
#endif
    } else if (_WTA_K == 3) {
      for (int i = 0; i < _dsize; ++i, _pattern += 12) {
        int t0, t1, t2, val;
        t0 = GET_VALUE(0);
        t1 = GET_VALUE(1);
        t2 = GET_VALUE(2);
        val = t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0);

        t0 = GET_VALUE(3);
        t1 = GET_VALUE(4);
        t2 = GET_VALUE(5);
        val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 2;

        t0 = GET_VALUE(6);
        t1 = GET_VALUE(7);
        t2 = GET_VALUE(8);
        val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 4;

        t0 = GET_VALUE(9);
        t1 = GET_VALUE(10);
        t2 = GET_VALUE(11);
        val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 6;

        _desc[i] = (uchar) val;
      }
    } else if (_WTA_K == 4) {
      for (int i = 0; i < _dsize; ++i, _pattern += 16) {
        int t0, t1, t2, t3, u, v, k, val;
        t0 = GET_VALUE(0);
        t1 = GET_VALUE(1);
        t2 = GET_VALUE(2);
        t3 = GET_VALUE(3);
        u = 0, v = 2;
        if (t1 > t0)
          t0 = t1, u = 1;
        if (t3 > t2)
          t2 = t3, v = 3;
        k = t0 > t2 ? u : v;
        val = k;

        t0 = GET_VALUE(4);
        t1 = GET_VALUE(5);
        t2 = GET_VALUE(6);
        t3 = GET_VALUE(7);
        u = 0, v = 2;
        if (t1 > t0)
          t0 = t1, u = 1;
        if (t3 > t2)
          t2 = t3, v = 3;
        k = t0 > t2 ? u : v;
        val |= k << 2;

        t0 = GET_VALUE(8);
        t1 = GET_VALUE(9);
        t2 = GET_VALUE(10);
        t3 = GET_VALUE(11);
        u = 0, v = 2;
        if (t1 > t0)
          t0 = t1, u = 1;
        if (t3 > t2)
          t2 = t3, v = 3;
        k = t0 > t2 ? u : v;
        val |= k << 4;

        t0 = GET_VALUE(12);
        t1 = GET_VALUE(13);
        t2 = GET_VALUE(14);
        t3 = GET_VALUE(15);
        u = 0, v = 2;
        if (t1 > t0)
          t0 = t1, u = 1;
        if (t3 > t2)
          t2 = t3, v = 3;
        k = t0 > t2 ? u : v;
        val |= k << 6;

        _desc[i] = (uchar) val;
      }
    } else {
      CV_Error(cv::Error::StsBadSize, "Wrong WTA_K. It can be only 2, 3 or 4.");
    }
#undef GET_VALUE
  }

  int max_features_;
  float scale_factor_;
  int pyramid_level_;
  int edge_threshold_;
  int wta_k_;
  int first_level_;
  int score_type_;
  int patch_size_;
};  /** End of class: orb extractor */

} /** End of namespace: EyeMARS */
#endif /// EYEMARS_FAST_CORNER_EXTRACTOR_H_
