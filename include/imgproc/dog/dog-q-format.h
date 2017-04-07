#ifndef EYEMARS_DOG_Q_H_
#define EYEMARS_DOG_Q_H_
#include <demo-parameters/parameters.h>
#include <cstring>          /// To use: memcpy()
#include <opencv2/imgproc/imgproc.hpp> /// To use: Mat
#ifdef __ARM_NEON__
#include <arm_neon.h>       /// To use: register asm()
#endif
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
/** 
 *	Author: Da Teng
 *	Update Date: 05/04/2015
 */
namespace EyeMARS {
void QDoGOctave_C(const cv::Mat& img, cv::Mat* gaussian_pyramid, cv::Mat* laplacian_pyramid, int h, int w);
void QDoG_u8_u16_C(unsigned char* src, unsigned short* dst, int h, int w);
void QDoG_u16_u16_C(unsigned short* src, unsigned short* dst, int h, int w);
void QDoG_Subtract_C(unsigned short* src, unsigned short* dst, short* diff, int h, int w);
void QDoG_Downsample_C(unsigned short* src, unsigned short* dst, int h, int w);

#ifdef __ARM_NEON__
// ARM-NEON implementation of DoG Pyramid using Q format
void QDoGOctave(cv::Mat* gaussian_pyramid, cv::Mat* laplacian_pyramid, int h, int w);
// ARM-NEON w/ initial image
void QDoGOctave(const cv::Mat& image, cv::Mat* gaussian_pyramid, cv::Mat* laplacian_pyramid, int h, int w);
/// ARM_NEON implementation of DoG using Q format: uchar src to short dst
void QDoG_u8_u16(unsigned char* src, unsigned short* dst, short* diff, int h, int w);
/// ARM_NEON implementation of DoG Stripe using Q format: uchar src to short dst
void QDoGStripe_u8_u16(unsigned char* src, unsigned short* dst, short* diff,
                       int numofiterations, int step1, int step2, int step3, int w);
/// ARM_NEON implementation of DoG using Q format: 
void QDoG_u16_u16(unsigned short* src, unsigned short* dst, int h, int w);
/// ARM-NEON implementation of DoGStripe using Q format: short src to short dst
void QDoGStripe_u16_u16(unsigned short* src, unsigned short* dst,
                      int numofiterations, int step1, int w);
/// ARM_NEON implementation of DoG with down sample using Q format:
void QDoGDownSample_u16_u16(unsigned short* src, unsigned short* dst_ds, unsigned short* dst, int h, int w);
/// ARM-NEON implementation of DoGStripe with down sample using Q format: short src to short dst
void QDoGDownSampleStripe_u16_u16(unsigned short* src, unsigned short* dst_ds, unsigned short* dst,
                      int numofiterations, int step, int step2, int w);
/// ARM-NEON implementation of createing image pyramid of DoG
void QDoGSubtract(unsigned short* src1, unsigned short* src2, short* diff, int h, int w);
/// ARM-NEON implementation of creating image pyramid of DoG
void QDoGSubtractStripe(unsigned short* src1, unsigned short* src2, short* diff, int step, int step2, int n_of_iterations, int w); 
/// ARM-NEON implementation of createing image pyramid of DoG
void QDoGSubtract_u8_u16(unsigned char* src1, unsigned short* src2, short* diff, int h, int w);  
/// ARM-NEON implementation of creating image pyramid of DoG
void QDoGSubtractStripe_u8_u16(unsigned char* src1, unsigned short* src2, short* diff, int step, int step2, int step3, int n_of_iterations, int w);                   
/// ARM-NEON implementation of downsampling
void QDoGDownsample(unsigned short* src, unsigned short* dst, int h, int w);
/// ARM-NEON implementation of downsampling a stripe of result
void QDoGDownsampleStripe(unsigned short* src, unsigned short* dst, int step, int step_dst, int w, int n_of_iterations);
/// ARM_NEON implementation of DoG with down sample using Q format: horizontal stripe
void QDoGDownsample_horizontal(unsigned short* src, unsigned short* dst, int h, int w);
/// ARM-NEON implementation of DoGStripe with down sample using Q format: short src to short dst
void QDoGDownSampleStripe_horizontal(unsigned short* src, unsigned short* dst, int w, int numofiterations);
/// ARM-NEON implementation of subtraction of 2 images in DoG using horizontal stripe
void QDoGSubtract_horizontal(unsigned short* src1, unsigned short* src2, short* diff, int h, int w);
/// ARM-NEON implementation of horizontal stripe for subtraction of 2 images in DoG
void QDoGSubtractStripe_horizontal(unsigned short* src1, unsigned short* src2, 
							unsigned short* src1_line2, unsigned short* src2_line2, short* diff, short* diff_line2, int n_of_iterations, int w);


///ARM-NEON implementation of DoG based on PyramidDown function
void DoG(unsigned char* src, unsigned char* dst, short* diff, int s, int h, int w);
#endif

#ifdef __ARM_NEON__
/// ARM-NEON implementation of constructing one strip of pyramid for pyramid down
void DoGStripe(unsigned char *src, unsigned char *dst, short* diff,
                      unsigned int num_of_iterations, 
                      unsigned int step, int step2, int step3, int w);
/// ARM-NEON implementation of creating image pyramid of DoG
void DoGSubtract(unsigned char* src, unsigned char* src2, short* diff, int h, int w);
/// ARM-NEON implementation of creating a stripe of DoG
void DoGSubtractStripe(unsigned char* src1, unsigned char* src2, short* diff, int step, int step2, int n_of_iterations, int w);

void DoG_twice(unsigned char* src, unsigned char* dst, short* diff, int s, int h, int w);
void DoGStripe_twice(unsigned char *src, unsigned char *dst, short* diff,
                      unsigned int num_of_iterations, 
                      unsigned int step, int step2, int step3, int w);
/// ARM-NEON implementationof create a pyramid of DoG
void DoGOctave(cv::Mat* gaussian_pyramid, cv::Mat* laplacian_pyramid, int w, int h);
/// ARM-NEON implementation of downsampling
void DoGDownsample(unsigned char* src, unsigned char* dst, int h, int w);
/// ARM-NEON implementation of downsampling a stripe of result
void DoGDownsampleStripe(unsigned char* src, unsigned char* dst, int step, int step_dst, int w, int n_of_iterations);
#endif  /** End of __ARM_NEON__ */

} /** End of namespace: EyeMARS */

#endif  // EYEMARS_DOG_Q_H_
