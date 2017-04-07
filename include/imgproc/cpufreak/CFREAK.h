#ifndef CPU_FREAK_EXTRACT_H
#define CPU_FREAK_EXTRACT_H

#include <vector>
#include <string>
// #include <gpu-freak-descriptor/cuda_FREAK.h>
// #include <gpu-freak-descriptor/CUDA_DoG.h>
#include <opencv2/core/core.hpp>
// #include <gpu-freak-descriptor/cuda_FREAK.h>
#include <opencv2/features2d/features2d.hpp>

 // #define HAVE_CUDA

class FREAKFeatures{
   public:
   	FREAKFeatures(int _width, int _height, int max_features, bool _rotation_invariance);
   	virtual ~FREAKFeatures();

   	void Extract(cv::Mat & _imagecv, std::vector<cv::KeyPoint> & _keypoints,  cv::Mat & _descriptors);

   private:
   	// GPUFREAKDescriptors gpu_descriptors_;	
   	// GPUDoGKeypoints gpu_dog_keypoints_;

   	cv::Mat gaussian_pyramid_cpu_[6];
   	cv::Mat laplacian_pyramid_cpu_[4];
   	int max_num_features_;
	   bool rotation_invariance_;
};

#endif

