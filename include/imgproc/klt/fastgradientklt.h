#ifndef EYEMARS_FAST_GRADIENT_KLT_H_
#define EYEMARS_FAST_GRADIENT_KLT_H_
#ifdef __ARM_NEON__
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/// ARM-NEON implementation of computing one strip of an approximation of the gradient
  inline void FastGradientKLT_13(short * patch, int load_step, short * gradient,
      float * matrix, int iterations_num);
/// ARM-NEON implementation of computing one strip of an approximation of the Sobel gradient
  void FastGradientKLT(short * patch, int height, int width,
      short * gradient, float * matrixA);

} /** End of namespace: EyeMARS */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_FAST_GRADIENT_KLT_H_
