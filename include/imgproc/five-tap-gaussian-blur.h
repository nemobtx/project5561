#ifndef _FIVE_TAB_GAUSSIAN_TAB_
#define _FIVE_TAB_GAUSSIAN_TAB_

#ifdef __ARM_NEON__
#include <arm_neon.h>       /// To use: register asm()
#endif

#include "opencv2/imgproc/imgproc.hpp" /// To use: register Mat, GaussianBlur
#include "core/matrix2d.h"   /// To use: Matrix2d

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {
/// Implementation of 5-tap Gaussian blur using EyeMARS datatypes
void FiveTapGaussianBlur(Matrix2d<unsigned char> & src, Matrix2d<unsigned char> & dst);
/// Implementation of 5-tap Gaussian blur on memory pointers.
void FiveTapGaussianBlur(unsigned char *src, unsigned char *dst, int h, int w);
#ifdef __ARM_NEON__
void FiveTapGaussianBlur_Vertical_2(unsigned char *src, unsigned char * dst,
                                    int img_height, int img_width);
#endif
} /** End of namespace: EyeMARS */

#endif  /// EYEMARS_PYRAMID_DOWN_H_
