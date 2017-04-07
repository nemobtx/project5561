#ifndef EYEMARS_BINARY_VECTOR_H_
#define EYEMARS_BINARY_VECTOR_H_

#include <Eigen/Core>  /// To use: Matrix

/** @namespace EyeMARS
 * The EyeMARS namespace.
 */
namespace EyeMARS {

typedef Eigen::Matrix<unsigned char, 1, Eigen::Dynamic> BinaryVector;

}  /** End of namespace: EyeMARS */

#endif /// EYEMARS_BINARY_VECTOR_H_
