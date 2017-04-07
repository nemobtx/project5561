#ifndef EYEMARS_HARRIS_RESPONSE_H
#define EYEMARS_HARRIS_RESPONSE_H

#ifdef __ARM_NEON__
#include <iostream>   /// To use: cout

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {
/// ARM-NEON implementation of computing one strip of the harris response of each pixel
void HarrisResponseStrip(unsigned char* src, float* dst, int height, int width, int dst_width, float* max_eigen);
/// ARM-NEON implementation of computing the harris response of each pixel
void HarrisResponse(unsigned char *src, float* dst, int height, int width, float* max_eigen);
} /** End of namespace: EyeMARS */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_HARRIS_RESPONSE_H
