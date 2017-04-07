#ifndef EYEMARS_PYRAMID_DOWN_H_
#define EYEMARS_PYRAMID_DOWN_H_

#ifdef __ARM_NEON__
#include <arm_neon.h>       /// To use: register asm()
#define  CV_DESCALE(x,n)  (((x) + (1 << ((n)-1))) >> (n))
#endif

#ifndef __ARM_NEON__
#include "opencv2/imgproc/imgproc.hpp" /// To use: register Mat, pyrDown()
#endif

#include "core/matrix2d.h"   /// To use: Matrix2d
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/// Implementation of pyramid down using EyeMARS datatypes
void PyramidDown(Matrix2d<unsigned char> & src, Matrix2d<unsigned char> & dst);
/// Implementation of pyramid down
void PyramidDown(unsigned char *src, unsigned char *dst, int h, int w);

#ifdef __ARM_NEON__
/// ARM-NEON implementation of constructing one strip of pyramid for pyramid down
void PyramidDownStrip(unsigned char *src, unsigned char *dst, 
                      unsigned int num_of_iterations, 
                      unsigned int step, int step2, int w);
/// ARM-NEON implementation of vertical convolution
void PyramidDownVertical(unsigned short *row0, unsigned short *row1, 
                         unsigned short *row2, unsigned short *row3, 
                         unsigned short *row4, unsigned char *dst,int width);
#endif  /** End of __ARM_NEON__ */

} /** End of namespace: EyeMARS */

#endif  /// EYEMARS_PYRAMID_DOWN_H_
