// Sub-pixel refinement for DoG Keypoints
//
// Input: DoG images and estimated extrema
// Output: Refined extrema
//
// Written by: John O'Leary (olear121@umn.edu)
// Last edited: August 17, 2014 
#ifndef __DOG_SUBPIXEL_REFINEMENT_H__
#define __DOG_SUBPIXEL_REFINEMENT_H__

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
// #include "dog-shared.h"

const int NUM_DOG_IMAGE = 2;  // number of DOG images EACH octave
const int NUM_OCTAVES = 2;    // number of octave
const int NUM_SCALES = NUM_DOG_IMAGE * NUM_OCTAVES;
const int MAX_SUBPIXEL_UPDATE_DISTANCE = 1;
const float SUBPIXEL_THRES_r = 10;
const float SUBPIXEL_THRES_ratio = (SUBPIXEL_THRES_r + 1)
    * (SUBPIXEL_THRES_r + 1) / SUBPIXEL_THRES_r;
const int LAPLACIAN_THRESHOLD = 2;


#define ONE_OVER_256 0.00390625
/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
// namespace EyeMARS {
  
bool Solve3by3System(const Eigen::Matrix<float, 3, 3>& A,
                     const Eigen::Matrix<float, 3, 1>& b,
                     Eigen::Matrix<float, 3, 1>* x);
bool ElimiateEdgeResponses(const Eigen::Matrix<float, 3, 3>& A,
                          float& score);

inline float GetVal(short shifted_val) {
  return static_cast<float>(shifted_val)*ONE_OVER_256;
}

template<typename precision>
float InterpolatePixelBilinear(const cv::Mat& image, float x, float y) {
  assert(image.data);
  int xp, yp;
  int xp_plus_1, yp_plus_1;
  float w0, w1, w2, w3;
  const precision* p0;
  const precision* p1;
  float result;

  xp = (int) x;
  yp = (int) y;

  xp_plus_1 = xp+1;
  yp_plus_1 = yp+1;

  p0 = image.ptr<precision>(yp);
  assert(p0);
  p1 = image.ptr<precision>(yp_plus_1);
  assert(p1);

  w0 = (xp_plus_1 - x)  * (yp_plus_1 - y);
  w1 = (x         - xp) * (yp_plus_1 - y);
  w2 = (xp_plus_1 - x)  * (y         - yp);
  w3 = (x         - xp) * (y         - yp);

  result = w0*p0[xp] + w1*p0[xp_plus_1] + w2*p1[xp] + w3*p1[xp_plus_1];

  return result;
}

template <typename Precision>
bool FindGradientAndHessian(Eigen::Matrix<float, 3, 3>& A,
                            Eigen::Matrix<float, 3, 1>& b,
                            cv::Mat lap[], short x, short y, 
                            cv::KeyPoint& kp) {
  cv::Mat lap0 = lap[(int)kp.size-1];
  cv::Mat lap1 = lap[(int)kp.size];
  cv::Mat lap2 = lap[(int)kp.size+1];
  
  float Dx, Dy, Ds, Dxx, Dxy, Dyy, Dxs, Dys, Dss;

  // Start easy with the scale-independent calculations
  // All based on the approximation of the Laplacian
  // Uses kernel [-1, 0, 1] for gradient
  // Uses kernel [1, -2, 1] for Hessian  
  Dx = 0.5*(lap1.at<Precision>(y,x+1) - lap1.at<Precision>(y,x-1));
  Dy = 0.5*(lap1.at<Precision>(y+1,x) - lap1.at<Precision>(y-1,x));
  Dxx = lap1.at<Precision>(y,x+1) - 2.0*lap1.at<Precision>(y,x) 
        + lap1.at<Precision>(y,x-1);
  Dyy = lap1.at<Precision>(y+1,x) - 2.0*lap1.at<Precision>(y,x) 
        + lap1.at<Precision>(y-1,x);
  Dxy = 0.25*((lap1.at<Precision>(y-1, x-1) + 
               lap1.at<Precision>(y+1, x+1)) -
              (lap1.at<Precision>(y-1, x+1) + 
               lap1.at<Precision>(y+1, x-1)));
  
  // The harder stuff, which might require down sampling, etc.
  if (lap0.size().width > lap1.size().width) {
    // Case where the bottom layer is larger than the next two
    // Upsample the center point
    float a = 2.0; // Instead of 1<<octave
    float b = 0.5; // Instead of pow(2.0, octave-1)-0.5
    float x_up = x*a;// + b;
    x_up += b;
    float y_up = y*a + b;
    
    float lower_center = InterpolatePixelBilinear<short>(lap0, x_up, y_up);

    Ds = 0.5*(lap2.at<Precision>(y,x) - lower_center);
    Dss = lower_center - 2.0*lap1.at<Precision>(y,x) + lap2.at<Precision>(y,x);
    Dxs = 0.25*((InterpolatePixelBilinear<short>(lap0, x_up-2, y_up) 
                + lap2.at<Precision>(y, x+1))
               - (InterpolatePixelBilinear<short>(lap0, x_up+2, y_up) 
                  + lap2.at<Precision>(y, x-1)));
    Dys = 0.25*((InterpolatePixelBilinear<short>(lap0, x_up, y_up-2) 
                + lap2.at<Precision>(y+1, x))
               - (InterpolatePixelBilinear<short>(lap0, x_up, y_up+2) 
                  + lap2.at<Precision>(y-1, x))); 
  } else if (lap1.size().width > lap2.size().width) {
    // Case where the top layer is smaller than the others
    // Down sample the center point
    float a = 0.5; // to bring it just one level more coarse
    float b = 0.5*a - 0.5;
    float x_down = x*a + b; // round with the 0.5
    float y_down = y*a + b; // round with the 0.5
        
    Precision upper_center = InterpolatePixelBilinear<short>(lap2, x_down, y_down);

    Ds = 0.5*(upper_center - lap0.at<Precision>(y, x));
    Dss = lap0.at<Precision>(y, x) - 2.0*lap1.at<Precision>(y, x) + upper_center;
    Dxs = 0.25*((lap0.at<Precision>(y, x-1) 
                 + InterpolatePixelBilinear<short>(lap2, x_down+.5, y_down))
                -(lap0.at<Precision>(y, x+1)
                  + InterpolatePixelBilinear<short>(lap2, x_down-.5, y_down)));
    Dys = 0.25*((lap0.at<Precision>(y-1, x) 
                 + InterpolatePixelBilinear<short>(lap2, x_down, y_down+.5))
                -(lap0.at<Precision>(y+1, x)
                  + InterpolatePixelBilinear<short>(lap2, x_down, y_down-.5)));
  } else if (lap1.size() == lap0.size() && 
             lap1.size() == lap2.size()) {
    // Case that all layers are equal
    Ds = 0.5*(lap2.at<Precision>(y,x) - lap0.at<Precision>(y,x));
    Dss = lap0.at<Precision>(y,x) 
          - 2.0*lap1.at<Precision>(y,x) 
          + lap2.at<Precision>(y,x);
    Dxs = 0.25*(lap0.at<Precision>(y, x-1) + lap2.at<Precision>(y, x+1) 
                - (lap0.at<Precision>(y, x+1) + lap2.at<Precision>(y, x-1)));
    Dys =  0.25*(lap0.at<Precision>(y-1, x) + lap2.at<Precision>(y+1, x) 
                - (lap0.at<Precision>(y+1, x) + lap2.at<Precision>(y-1, x)));
  } else {
    std::cout << "Improperly sized DoGs.\n";
    return false;
  }
  
  // Gradient
  b << -Dx, -Dy, -Ds;
  // Hessian
  A << Dxx, Dxy, Dxs,
       Dxy, Dyy, Dys,
       Dxs, Dys, Dss;
  
  return true;
}

#define kFREAK_NB_SCALES 64
#define kFREAK_SMALLEST_KP_SIZE  7

static const float kFREAK_PATTERN_SCALE = 16.0;
static const float kFREAK_BIG_RADIUS = 2.0 / 3.0;
static const double kFREAK_LOG2 = 0.693147180559945;

inline int GetFREAKScaleIndex(float size,
                                                  int num_octaves) {
  float size_cst =
    static_cast<float>(kFREAK_NB_SCALES / (kFREAK_LOG2 * num_octaves));
  int res = static_cast<int>(logf(size / kFREAK_SMALLEST_KP_SIZE) *
                             size_cst + 0.5f);
  if (res < 0)
    res = 0;
  if (res >= kFREAK_NB_SCALES)
    res = kFREAK_NB_SCALES-1;
  return res;
}

inline bool BorderFilter(float size,
                                            int num_octaves,
                                            float xs, float ys, int im_width,
                                            int im_height) {

  int freak_scale_ind = GetFREAKScaleIndex(size, num_octaves);
  const float scale_step = pow(2.0f, static_cast<float>(num_octaves) / 
                                kFREAK_NB_SCALES);
  float scaling_factor = powf(scale_step, freak_scale_ind);
  int freak_radius = (static_cast<int>(ceil((1.5f * kFREAK_BIG_RADIUS) *
                                           scaling_factor * 
                                           kFREAK_PATTERN_SCALE)) + 1 );
  
  if (xs <= freak_radius || ys <= freak_radius ||
      xs >= im_width - freak_radius || ys >= im_height - freak_radius) {
    return false;
  }

  return true;
}


// Computes 
template <typename Precision>
void SubpixelRefinement(cv::Mat lap[],
        std::vector<cv::KeyPoint>& keypoints) {
  int num_keypoints = 0; // increments as each point qualifies

  const static float k = pow(2.f, 1.f / (3 - 1));
//   string filename;
//   string filename_in;
// #ifdef ANDROID 
//   // Android
//   filename = "/data/data/test_freak/param_nof.txt";
// #else 
//   // Linux
//   filename = "/home/da/dev/MarsFramework/MARSFramework/build/param_nof.txt";
// #endif


//   ofstream output(filename, ofstream::app | ofstream::in);
//   ifstream input(filename);

  int ctn_keypoints = -1;

  for (int i=0;i < keypoints.size();i++) {
    // sanity tests
    cv::KeyPoint & kp = keypoints[i];

    assert(kp.size > 0);
    assert(kp.size < NUM_SCALES);
    // TEST subpixel refinement
    ctn_keypoints += 1;
    // cout << "key point index: " << ctn_keypoints << endl;
    // field 1:
    // End TEST subpixel refinement

    // output << "Idx: " << ctn_keypoints << " ";

    short x = kp.pt.x, y = kp.pt.y;

    // Down sample the point if need be
    if (kp.octave) {
      float a = 1.0/(1<<kp.octave); // for a power of two
      float b = 0.5*a - 0.5;
      // field 2:
      // output << "kp.octave TRUE: ";
      // output << a << " ";
      // output << b << " ";
      
      x = static_cast<short>(kp.pt.x*a + b + 0.5); // round with the 0.5
      y = static_cast<short>(kp.pt.y*a + b + 0.5); // round with the 0.5
      // output << x << " ";
      // output << y << " ";
    }

    
    // // Form gradient and Hessian
    Eigen::Matrix<float, 3, 3> A;
    Eigen::Matrix<float, 3, 1> b;
    if (!FindGradientAndHessian<Precision>(A, b, lap, x, y, kp)) {
      // output << "FG FALSE: ";
      // output << "Get log: Find Gradient FALSE" << endl;
      // output << endl;
      continue;
    }

    // Solve A*x=b
    Eigen::Matrix<float, 3, 1> v;
    if (!Solve3by3System(A, b, &v)) {
      // output << "Get log: Solve3by3System FALSE" << endl;
      // output << endl;
      continue;
    }
  
    // Discard a point if the update distance is too much
    // since the point is likely unstable
    if(abs(v[0]) > MAX_SUBPIXEL_UPDATE_DISTANCE || abs(v[1]) > MAX_SUBPIXEL_UPDATE_DISTANCE) {
      // output << "Get log: MAX Distance FALSE" << endl;
      // output << endl;
      continue;
    }
    float edge_score;
    // Check edge score
    if (!ElimiateEdgeResponses(A, edge_score)) {
      // output << "Get log: EliminateEdgeResponses FALSE" << endl;
      // output << endl;
      continue;
    }
    
    // Got through all the testing, now the keypoint is the one that get actually log
    // output << "Get log: TRUE" << " ";


    // Compute an estimated laplacian score at subpixel point
    kp.response = abs(lap[(int)kp.size].at<Precision>(y, x) - (b[0]*v[0] + b[1]*v[1] + b[2]*v[2]));
    
    kp.pt.x = kp.pt.x + v[0]*(1<<kp.octave);
    kp.pt.y = kp.pt.y + v[1]*(1<<kp.octave);
    kp.size = kp.size + v[2];    
    
    if (abs(edge_score) < SUBPIXEL_THRES_ratio &&
           kp.response > LAPLACIAN_THRESHOLD &&
           kp.pt.x >= 0 &&
           kp.pt.x < lap[0].cols &&
           kp.pt.y >= 0 &&
           kp.pt.y < lap[0].rows &&
           kp.size >= 0 &&
           kp.size < NUM_SCALES) {

      float size = 7*powf(k, kp.size)*(1<<kp.octave);
      
      if (!BorderFilter(size, 2, kp.pt.x, kp.pt.y, lap[0].cols, 
                     lap[0].rows))
          continue;

      // compute effective sigma 
      kp.size = size;
      keypoints[num_keypoints++] = kp;
    }
    // output << endl;
  }
  
  // output.close();
  // input.close();
  keypoints.resize(num_keypoints);
}

// } // namespace EyeMARS
#endif // __DOG_SUBPIXEL_REFINEMENT_H__
