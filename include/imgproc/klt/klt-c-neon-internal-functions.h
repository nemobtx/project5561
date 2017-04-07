#ifndef EYEMARS_KLT_C_NEON_INTERNAL_FUNCTIONS_H_
#define EYEMARS_KLT_C_NEON_INTERNAL_FUNCTIONS_H_

namespace EyeMARS {

/// KLT Patch extraction with bilinear interpolation of pixels
void KltExtractPatch(short* weights,
                  unsigned char* src, short* dst,
                  int img_width);
/// KLT Patch extraction with bilinear interpolation of pixels
void KltExtractPatch2(short* weights,
                   unsigned char* image2, int img_width,
                   short* patch2, short* patch1,
                   short* gradient, float* vectorb);

/// KLT Computing an approximation of the gradient of the passed patch
void KltGradient(short * patch,
                     int height, int width,
                     short * gradient, float * matrixA);

void FastGradientKLT_new(short * patch, int height, int width,
   short * gradient, float * matrixA);

}  // End of namespace: EyeMARS
#endif  // End of file: EYEMARS_KLT_C_NEON_INTERNAL_FUNCTIONS_H_
