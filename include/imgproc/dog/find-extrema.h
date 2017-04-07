#ifndef FIND_EXTREMA_LEE
#define FIND_EXTREMA_LEE
#include "core/keypoint.h"
#include <opencv2/features2d/features2d.hpp>

namespace EyeMARS {
/*bilinear interolation*/
// float interpolate_pixel_bilinear(const cv::Mat& image, float x, float y);
/*functino for finding extrema*/
void FindExtrema(const cv::Mat * laplacian_pyramid, std::vector<Keypoint> & keypoints, double laplacian_threshold);
/*function for upsampling the pixel postion to orignal size image*/
// void upsample_point_bilinear(float& xp, float& yp, float x, float y, int octave);
//bool compare_x(const keypoint& a, const keypoint& b); // USELESS, THIS FUNCTION IS FOR SORTING VECTOER
/*functoin for reading the pixel value from image txt file*/

void FindExtrema(const cv::Mat * laplacian_pyramid, std::vector<cv::KeyPoint> & keypoints, double laplacian_threshold );

void read_image(cv::Mat * laplacian_pyramid);
/*function for checking the results*/
void check_feature(const std::vector<Keypoint> & keypoints);

}
#endif
