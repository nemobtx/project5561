#ifndef EYEMARS_SHARR_GRADIENT_H
#define EYEMARS_SHARR_GRADIENT_H

#ifdef __ARM_NEON__
#define CACHE_LINE_SIZE 64
#endif

#ifndef __ARM_NEON__
#include "opencv2/imgproc/imgproc.hpp" /// To use: register Mat, Sharr()
#endif

#include "core/matrix2d.h"   /// To use: Matrix2d
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/// Implementation of Sharr gradient using EyeMARS datatypes
void SharrGradient(Matrix2d<unsigned char> & src, Matrix2d<short> & dst);
/// Implementation of Sharr gradient
void SharrGradient(unsigned char* src, short* dst, int height, int width);

#ifdef __ARM_NEON__
/// ARM-NEON implementation of computing one strip of Sharr gradient 
void SharrGradientStrip(unsigned char* src, short* dst, int height, int width);
#endif

}
/** End of namespace: EyeMARS */

#endif  /// EYEMARS_SHARR_GRADIENT_H_
