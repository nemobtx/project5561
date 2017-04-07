#ifndef EYEMARS_IMAGE_RESIZE_H_
#define EYEMARS_IMAGE_RESIZE_H_

#ifdef __ARM_NEON__
#include <arm_neon.h>       /// To use: register asm()
// #define  CV_DESCALE(x,n)  (((x) + (1 << ((n)-1))) >> (n))
#endif

#ifndef __ARM_NEON__
#include "opencv2/imgproc/imgproc.hpp" /// To use: register Mat, pyrDown()
#endif

// #include "core/matrix2d.h"   /// To use: Matrix2d
#include <cmath>
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {


#ifdef __ARM_NEON__

void NEON_operator_height_width_left_over(unsigned char * src, unsigned char * temp_result, int img_width, int img_height,
                  int real_height, float bilinear_mask1[][9], float  bilinear_mask2[][9],
                  float  bilinear_mask3[][9], float  bilinear_mask4[][9], 
                  int width_output, int output_height_coutner, int ii, int j);

void NEON_operator(unsigned char * src, unsigned char * dst, int img_width, int img_height,
                  int real_height, float bilinear_mask1[][9],  float bilinear_mask2[][9],
                  float bilinear_mask3[][9], float bilinear_mask4[][9], 
                  int width_output, int output_height_coutner, int ii, int j);

void img_resize_NEON(unsigned char * src,
                        unsigned char * dst, int img_width, int img_height);

// void img_resize_Cpp(unsigned char * src,
                        // unsigned char * dst, int img_width, int img_height);

#endif  /** End of __ARM_NEON__ */

} /** End of namespace: EyeMARS */

#endif  /// EYEMARS_IMAGE_RESIZE_H_
