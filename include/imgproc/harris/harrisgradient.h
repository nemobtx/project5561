#ifndef EYEMARS_HARRIS_GRADIENT_H_
#define EYEMARS_HARRIS_GRADIENT_H_

#ifdef __ARM_NEON__
#define __USE_EXACT_SOBEL__

#ifndef __USE_EXACT_SOBEL__
#include <iostream>
#define CACHE_LINE_SIZE 64
#endif

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

#ifndef __USE_EXACT_SOBEL__
/// ARM-NEON implementation of computing one strip of an approximation of the Sobel gradient
void HarrisGradientStrip(unsigned char* src, unsigned char* dst, int height, int width);
#endif

/// ARM-NEON implementation of computing an approximation of the Sobel gradient 
void HarrisGradient(unsigned char* src, unsigned char* dst, int height, int width);

} /** End of namespace: EyeMARS */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_HARRIS_GRADIENT_H_
