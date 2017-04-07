#ifndef EYEMARS_GRID_H_
#define EYEMARS_GRID_H_

#ifdef __ARM_NEON__
#include <vector>         /// To use: vector

#include "core/corner.h"  /// To use: Corner

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class Grid
 *  @brief The Grid class.
 *  The Grid class stores an indexed two dimensional array to facilitate 
 *  searching, among corners for the minimum distance supression of 
 *  candidate corners.
 */
class Grid {
 public:	
	Grid() : 
    width(0), height(0), cell_size(0), features() {}
	// Parametrized Constructor
	Grid(int _image_width, int _image_height, int _min_distance) : 
    width((_image_width + _min_distance - 1) / _min_distance), 
    height((_image_height + _min_distance - 1) / _min_distance), 
    cell_size(_min_distance), features(width * height) {}
  /// Destructor  
  ~Grid() {}
  /// Methods
  /// Resize the Grid
  inline void Resize(int image_height, int image_width) {	
		width = (image_width + cell_size - 1) / cell_size;
		height = (image_height + cell_size - 1) / cell_size;
    features.clear();
		features.resize(width * height);
  }  
  /// Setters
  // Set cell_size
  inline void setCellSize(int _min_distance) {
    cell_size = _min_distance;
  }  
  /// Data members
	int width;
	int height;
	int cell_size;
	std::vector<std::vector<Corner<short>>> features;
}; /** End of class: Grid */

}  /** End of namespace: EyeMARS */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_GRID_H_
