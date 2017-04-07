#ifndef EYEMARS_MIN_DISTANCE_H_
#define EYEMARS_MIN_DISTANCE_H_

#ifdef __ARM_NEON__
#include <algorithm>		/// To use: max(), min()  

#include "imgproc/harris/grid.h"  /// To use: Grid

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/// Implementation of Corner suppression based on the minimum distance between them
void MinDistanceSuppression(float* src, std::vector<const float*> tmp_corners, 
                            int height, int width, std::vector<Corner<short>> & corners,
                            Grid * grid, int min_distance, int max_corners, float* max_eigen = 0);
} /** End of namespace: EyeMARS */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_MIN_DISTANCE_H_
