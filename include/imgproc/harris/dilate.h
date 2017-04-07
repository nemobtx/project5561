#ifndef EYEMARS_DILATE_H_
#define EYEMARS_DILATE_H_

#ifdef __ARM_NEON__
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/// ARM-NEON implementation of computing one strip of dilation 
/// by convolving with maximum filter for non-maximum suppresion
void DilateStrip(float* src, float*dst, int height, int width, int dst_width, float& maximum);

/// Implementation of dilation by convolving with maximum filter 
///for non-maximum suppresion
void Dilate(float* src, int height, int width, float& maximum);
 
} /** End of namespace: EyeMARS */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_DILATE_H_
