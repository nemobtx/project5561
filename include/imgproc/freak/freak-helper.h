#ifndef __FREAK_HELPER_H_
#define __FREAK_HELPER_H_
#include "opencv2/core.hpp"
#include "core/keypoint.h"
#include "core/freak-feature.h"
#include "core/keypoint.h"
#include <memory>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>


#define SQRT2 1.41421356237309504880
#define SQRT1_2 0.70710678118654752440
#define LOG_SQRT2 0.346573590279973
#define kSampleLength 64
#define kNumScales 6
#define kLevelsPerOctave 3
#define kBaseSigma 0.0
#define kScaleFactor 2.885390


#define DO_ANGLES 1
#define USE_ARM_NEON_FORM_FREAK 1
#define USE_ARM_NEON_SAMPLE 1

namespace EyeMARS {
typedef std::vector<std::shared_ptr<FreakFeature>> FREAKPtrVector;

bool QualifiedKeypoint(const Keypoint& kp, float max_radius, int img_width,
                       int img_height);
bool QualifiedKeypoint(const cv::KeyPoint& kp, float max_radius, int img_width,
                       int img_height);
/// Computes values or each sample around a keypoint
void SamplePoint(unsigned short samples[], const std::vector<cv::Mat>& pyr,
                 const Keypoint& kp, float first_offset, float first_sigma,
                 float R[]);

void SamplePoint(unsigned short samples[kSampleLength],
                 const std::vector<cv::Mat>& pyr, const cv::KeyPoint & kp,
                 float first_offset, float first_sigma, float R[4]);

void SampleOddRing(unsigned short* samples, const std::vector<cv::Mat>& pyr,
                   float x, float y, float radius, float sigma, float R[4], float logSigma);

void SampleEvenRing(unsigned short* samples, const std::vector<cv::Mat>& pyr,
                    float x, float y, float radius, float sigma, float R[4], float logSigma);

void SampleCenter(unsigned short* samples, const std::vector<cv::Mat>& pyr,
                  float x, float y, float sigma, float R[4], float logSigma);

void GetCoordinates(float x, float y, float sigma, int* octave, int* level,
                    float* relx, float* rely/*, float* relsigma*/, float logSigma);

/// Compares the samples points to create a binary descriptor
std::shared_ptr<FreakFeature> CreateFreakDescriptor(unsigned short samples[],
                                                    const Keypoint& kp);

// void CreateFreakDescriptor(unsigned short samples[], const Keypoint& kp, std::shared_ptr<FreakFeature> *freak_ptr);
void CreateFreakDescriptor(
    unsigned short samples[kSampleLength],  cv::Mat & _descriptor, int _index);

void DownsampleBilinear(float x, float y, float s, int octave, float* px,
                        float* py/*, float* rs*/);

void CompareAllPairs(unsigned char* bitvector, int pos,
                     const unsigned short* samples);
void CompareOneToOne(unsigned char* bitvector, int pos,
                     const unsigned short* samples0,
                     const unsigned short* samples1, int offset);

void CompareAllAgainstAll(unsigned char* bitvector, int pos,
                          const unsigned short* samples0,
                          const unsigned short* samples1);

void BitVectorSet(unsigned char* bitvector, int pos, unsigned char val);

void CompareOneAgainstAll(unsigned char* bitvector, int pos,
                          const unsigned short sample,
                          const unsigned short* samples);
/// The main FREAK call
FREAKPtrVector RunFreak(const std::vector<cv::Mat>& gauss_pyr,
                        const std::vector<Keypoint>& keypoints, bool do_angle);

void RunFreak(const std::vector<cv::Mat>& gauss_pyr, const std::vector<cv::KeyPoint>& kps,
                          std::vector<cv::KeyPoint>& keypoints, cv::Mat & _descriptor, bool do_angle);


unsigned short SampleReceptor(const cv::Mat& img, int w, int h, float x,
                              float y, float dx, float dy, float radius,
                              float sigma, float R[4]);
void IC_AnglePreProc(int width, int height, int halfPatchSize, std::vector<int>& umax);
float IC_Angle(const cv::Mat& image, const int half_k, cv::Point2f pt,
                      const std::vector<int> & u_max);

void ReadGaussImage(cv::Mat gaussian_pyr[],
                      const std::vector<std::string>& filenames); // todo - don't use

void CreateFloatLaplacianPyr(cv::Mat* pyr);
void CreateShortLaplacianPyr(cv::Mat* pyr);
void CreateFloatGaussianPyr(cv::Mat* pyr);
void CreateUShortGaussianPyr(cv::Mat* pyr);
void ConvertBaseToFloat(cv::Mat* pyr);
void ReadGaussImage(const std::vector<std::string>& fnames,
                    cv::Mat * gauss_pyramid);
void ReadGaussImageUChar(const std::vector<std::string>& fnames,
                    cv::Mat * gauss_pyramid);
void ReadLapImage(const std::vector<std::string>& fnames,
                  cv::Mat * laplacian_pyramid);
void ReadLapImageShort(const std::vector<std::string>& fnames,
                  cv::Mat * laplacian_pyramid);
}
#endif // __FREAK_HELPER_H_