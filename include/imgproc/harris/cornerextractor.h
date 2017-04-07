#ifndef EYEMARS_CORNER_EXTRACTOR_H_
#define EYEMARS_CORNER_EXTRACTOR_H_

#ifdef __ARM_NEON__
#include <vector>		    // To use: vector 
#include <algorithm> 		// To use: sort()

#include "core/corner.h"                      /// To use: Corner
#include "imgproc/harris/harrisgradient.h"    /// To use: HarrisGradient()
#include "imgproc/harris/harrisresponse.h"    /// To use: HarrisResponse()
#include "imgproc/harris/dilate.h"		        /// To use: Dilate() 
#include "imgproc/harris/mindistance.h"	      /// To use: MinDistanceSuppression()

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/// Comparison between two float values
inline bool GreaterThanPtr(float* f1, float* f2) {
  return (*f1 > *f2);
}
/// The Main Harris Function
void CornerExtractor(unsigned char* image, unsigned char* gradient_result, 
                     float* harris_values, int height, int width,
                     std::vector<Corner<short>> & corners, Grid *grid, int min_distance = 10,
                     float percent = 0.006, int max_corners = 200, float* max_eigen = 0);

} /** End of namespace: eyemars */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_CORNER_EXTRACTOR_H_
