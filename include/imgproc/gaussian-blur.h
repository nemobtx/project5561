/*
 * gaussian-blur.h
 *
 *  Created on: Jun 20, 2014
 *      Author: mars
 */

#ifndef GAUSSIAN_BLUR_H_
#define GAUSSIAN_BLUR_H_

#include "core/matrix2d.h"   /// To use: Matrix2d


namespace EyeMARS {

void GaussianBlur(Matrix2d<unsigned char> & src, Matrix2d<unsigned char> & dst);
void GaussianBlur(unsigned char *src, unsigned char *dst, int h, int w);

#ifdef __ARM_NEON__
void GaussianBlurNeon(unsigned char* src, unsigned char* dst, int height, int width);
void GaussianBlurStrip(unsigned char* src, unsigned char* dst,
                        int w, int step, int numofiterations);
#endif


#ifndef __ARM_NEON__
void GaussianBlurC(unsigned char* src, unsigned char* dst, int height, int width);
#endif

}

#endif /* GAUSSIAN_BLUR_H_ */
