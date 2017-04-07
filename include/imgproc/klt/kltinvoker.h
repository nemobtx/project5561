#ifndef EYEMARS_KLT_INVOKER_H_
#define EYEMARS_KLT_INVOKER_H_
#ifdef __ARM_NEON__
#include <math.h>			  /// To use: round, floor, sqrt
#include <vector>			  /// To use: vector
#include <string.h>			/// To use: memset
#include <opencv2/video/tracking.hpp> /// To use: OPTFLOW_USE_INITIAL_FLOW

#include "core/corner.h"		              /// To use: Corner
#include "core/matrix2d.h"                /// To use: Matrix2d
#include "imgproc/klt/extractpatch.h"	    /// To use: extractPatch
#include "imgproc/klt/fastgradientklt.h"	///	To use: fastGradient

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

/** @class KltInvoker
 * 	@brief The KltInvoker class.
 * 	The class track the corners from image 1 to image 2.
 *  It is used to be called by TBB _parallel_for.
 */
class KltInvoker {
 public:
  /// Default Constructor should not be used!
	KltInvoker();
	/// Default Constructor
	KltInvoker(int _iterations_num, float _epsilon,
             float _min_eigen_threshold,
				     int _window_width, int _window_height,
             int _flags, unsigned char * _status,
             float * _error, float * _min_eigen, int _max_layers,
             Matrix2d<unsigned char> & _image1, Matrix2d<unsigned char> & _image2,
				     int _layer,
             std::vector<Corner<short>*> * _corners1,
             std::vector<Corner<short>*> * _corners2);
	/// Destructor
	~KltInvoker();
  /// Methods
	/// The Invoker operation function (Tracking Function)
	void operator() (int _start, int _end);
 private:
  /// Data Members
	int iterations_num_;			/// iterations_num: maximum number of refinement iterations.
	float epsilon_;	  			  /// epsilon: the convergence threshold.
	float min_eigen_threshold_;	/// min_eigen_threshold: minimum eigen value of cornerness to be accepted.

	int window_width_;			  /// window_width: extracted patch width.
	int window_height_;			  /// window_height: extracted patch height.

	int flags_;					        /// flags: operation falgs.
	unsigned char * status_;		/// status: track corner status (success, fail).
	float* error_;				  /// error: the absolute patch difference (i.e. abs(image1 - image2)).
	float* min_eigen_;			/// min_eigen: minimum eigen value of cornerness for corner in image 1.

	int max_layers_;			/// max_layers: maximum number of levels in the pyramid.
	Matrix2d<unsigned char> image1_;		      /// image1: current pyramid 1 level.
	Matrix2d<unsigned char> image2_;		      /// image2: current pyramid 2 level.

    int layer_;	     /// layer: current operation layer.

    std::vector<Corner<short>*> * corners1_;	  /// corners1: list of corners from image 1.
	std::vector<Corner<short>*> * corners2_;	  /// corners2: list of corners from image 2.

  Matrix2d<short> gradient_;    /// gradient_: memory container for the gradient.


};	/** End of class: KltInvoker */

} /** End of namespace: EyeMARS */
#endif  /** End of __ARM_NEON__ */

#endif  /// EYEMARS_KLT_INVOKER_H
