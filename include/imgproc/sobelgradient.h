#ifndef EYEMARS_SOBEL_GRADIENT_H_
#define EYEMARS_SOBEL_GRADIENT_H_

#ifdef __ARM_NEON__
#define CACHE_LINE_SIZE 64
#include <arm_neon.h>       /// To use: register asm()
#endif

#ifndef __ARM_NEON__
#include "opencv2/imgproc/imgproc.hpp" // To use: register Mat, Sobel()
#endif

#include "core/matrix2d.h"   /// To use: Matrix2d
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/// Implementation of Sobel gradient using eyeMARS datatypes
void SobelGradient(Matrix2d<unsigned char> & src, Matrix2d<signed char> & dst);
/// Implementation of Sobel gradient
void SobelGradient(unsigned char* src, signed char * dst, int height, int width);

#ifdef __ARM_NEON__
/// ARM-NEON implementation of computing one strip of Sobel gradient 
void SobelGradientStrip(unsigned char * src, signed char * dst, int height, int width);
#endif

}
/** End of namespace: EyeMARS */

#endif  /// EYEMARS_SOBEL_GRADIENT_H_
