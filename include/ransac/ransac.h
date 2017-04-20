#ifndef RANSAC_H_
#define RANSAC_H_

#include <vector>	/// To use: vector
#include "ransac-solver/ransac-solver.h"	/// To use: RansacSolver
/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {

/**	@class Ransac
 * 	@brief The Ransac class.
 *	This class implements the Ransac algorithm.
 */
class Ransac {
// -------------- Parameters -------------------
 private:
  RansacSolver* ransac_solver;	/// ransac_solver: Pointer to the Ransac solver

  // RANSAC algorithm parameters
  double prob_outlier;			/// The probability of outliers in the dataset.
  double size_bound_percent;	/// The percentage lower bound of the size of an acceptable consensus set of features (i.e. lower bound = size_bound_percnt * number of features).
  unsigned int max_iterations;	/// Max number of iterations the RANSAC algorithm took to estimate t0.
  // Output
  std::vector<int> hypothesis_generator;
  std::vector<int> inlier_index;  // The indices of the inlier features in the dataset.
  std::vector<int> outlier_index;
  bool max_reached;  // Boolean variable indicating that the RANSAC algorithm reached the maximum iterations

// -------------- Functions ---------------------
 public:
  // A. Constructor and Destructor
  Ransac();
  Ransac(double percent, unsigned int max_iter);
  virtual ~Ransac();

  // B. Setters
  inline void setOutlierProb(double p) {
    prob_outlier = p;
  }
  inline void setSizeBoundPercent(double percent) {
    size_bound_percent = percent;
  }
  inline void setMaxIterations(unsigned int max_iter) {
    max_iterations = max_iter;
  }
  inline void setRansacSolver(RansacSolver* _ransac_solver) {
    ransac_solver = _ransac_solver;
    inlier_index.clear();
    outlier_index.clear();
    max_reached = false;
  }

  // C. Getters
  inline double getOutlierProb() {
    return prob_outlier;
  }
  //points that generated the winning hypothesis
  std::vector<int> getHypothesisGenerators() {
    return hypothesis_generator;
  }
  inline double getSizeBoundPercent() {
    return size_bound_percent;
  }
  inline double getMaxIterations() {
    return max_iterations;
  }
  inline RansacSolver* getSolver() {
    return ransac_solver;
  }
  inline std::vector<int>& getInlierIndices() {
    return inlier_index;
  }
  inline std::vector<int>& getOutlierIndices() {
    return outlier_index;
  }
  /// @fn isMaxReached, Return whether the last run for the RANSAC
  /// algorithm reached the maximum number of iterations or not.
  inline bool isMaxReached() {
    return max_reached;
  }
  /// Operations Functions
  /// @fn Run, Run the RANSAC algorithm,
  /// and returns the number of iterations performed by the algorithm.
  int Run();

  /// @fn GetInliers, Classify the data point and get the indices of the inliers in "inlier_index".
  inline void GetInliers() {
    // Clear the old inliers before generating new inliers
    inlier_index.clear();
    outlier_index.clear();
    // Evaluate the inliers in the dataset based on it's operations and algorithm
    ransac_solver->GetInliers(inlier_index, outlier_index);
  }

  // F. Print and Log Results
  // [TODO] WE NEED THESE FUNCTIOSN BACK.
  // void Print();
  // void SaveResults(ostream& out);
  // void SaveResults(ostream& out);
};
/** End of class: Ransac */
} /** End of namespace: Ransac */
#endif	/** End of file: RANSAC_H_ */
