#ifndef _DOG_C_H_
#define _DOG_C_H_
#include <opencv2/core.hpp>
namespace EyeMARS {
/// Gaussian Blur using kernel [1 4 6 4 1]: from uchar src to float dst
void DoG_Blur_u8_f32(unsigned char* src, float* dst, int h, int w);

/// Gaussian Blur using kernel [1 4 6 4 1]: from float src to float dst
void DoG_Blur_f32_f32(float* src, float* dst, int h, int w);

/// Subtraction of 2 images
template<typename T>
void DoG_Subtract(T src, float* dst, float* diff, int h, int w) {
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			diff[i*w + j] = (float)dst[i*w + j] - (float)src[i*w + j];
		}
	}
}
/// Downsampling a image to 1/4 of its size
void DoG_Downsample(float* src, float* dst, int h, int w);


/// Create DoGOctave 
void DoGOctave_C(cv::Mat* gaussian_pyramid, cv::Mat* laplacian_pyramid, int h, int w);
void DoGOctave_C(const cv::Mat& img, cv::Mat* gaussian_pyramid, cv::Mat* laplacian_pyramid, int h, int w);
} // End of namespace EyeMARS

#endif