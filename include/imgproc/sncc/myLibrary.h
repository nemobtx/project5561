#ifndef LIB_SNCC
#define LIB_SNCC

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/QR>

// -- My library functions --
const float DEFAULT_EDGE_THRESHOLD = 0.1f;
const float DEFAULT_MAX_DISC_THRESHOLD = 0.2f;
const float DEFAULT_SIGMA_RANGE = 10.0f;

void calc_color_weighted_table(float* table_color, float sigma_range, int len);
void calc_space_weighted_filter(float* table_space, int radius);

// -- Writes a binary file to be read by MATLAB --
int WriteBinaryFile (float *file, char *fileName, int nColumns, int nRows);
int WriteTextFile (float *file, char *fileName, int nColumns, int nRows);
int WritePclFile (char *fileName, std::vector<std::vector<float>> &xyz);
int ReadTextFile (float *filedata, char *fileName, int nColumns, int nRows);

// -- Calculates disparity base on various methods --
void SNCC (unsigned char* leftImage, unsigned char* rightImage, unsigned char* disparityMap, float *disp, float* costMap, int nRows, int nColumns, int windowSize, int sumWindowSize, int dMin, int dMax);
void reprojectImageTo3D(unsigned char* leftImage, float* disp, std::vector<std::vector<float>> &xyz, int h, int w, cv::Matx44d cq);
void disp_bilateral_filter(float* disp, unsigned char* img, int h, int w,
			   const float* ctable_color, const float * ctable_space, size_t ctable_space_step,
			   int cradius, float cedge_disc, float cmax_disc);

#endif
