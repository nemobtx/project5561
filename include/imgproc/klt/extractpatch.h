#ifndef EYEMARS_EXTRACT_PATCH_H_
#define EYEMARS_EXTRACT_PATCH_H_
#ifdef __ARM_NEON__
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/// ARM-Neon implementation of patch extraction with bilinear interpolation of pixels
  void ExtractPatch(short* weights, unsigned char* src, short* dst, int img_width);

  void ExtractPatch2(short* weights, unsigned char* image2, int img_width,
      short* patch2, short* patch1, short* gradient,
      float* vectorb);
} /** End of namespace: EyeMARS */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_EXTRACT_PATCH_H_
