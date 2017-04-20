#ifndef CLASSIFYLINES_H_
#define CLASSIFYLINES_H_

#include <Eigen/Core> 
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>
#include "ransac-solver/ransac-solver.h"	/// To use: RansacSolver
#include "vanishing-points-hyp/utilities.h" 
#include "geometry/geometry.h"
#include "vanishing-points-hyp/vanishing-points-hyp.h" /// To use: GetHypVP, Hyp*
#include "vanishing-points-hyp/houseQR.h"
#include <profiler.h>
#define MINIMAL 0
#define RELAXED 1
#define NONRELAXED 2

/** @namespace Ransac
 * The Ransac namespace.
 */
namespace Ransac {
/**	@class ClassifyLines
 * 	@brief
 *	This class implements the lines classifier based Ransac.
 */
class ClassifyLines : public RansacSolver
{
public:

	Eigen::Matrix3Xf lines;		// Data lines input
	Eigen::Matrix4Xd unnormalized_lines;		// Data lines input
protected:
	// Data Members
	
	/// Loop control parameter:
	int kMaxHypotheses;
	int kMinHypotheses;
	double kInClassResidualThreshold;
	double kColinearityThreshold;

	double early_termination_percent;
	
	/// Input:
	// Eigen::Matrix3Xf lines;		// Data lines input
	// Eigen::Matrix4Xf unnormalized_lines;		// Data lines input
	
	/// Parameters:
	Eigen::Vector2d fc_;
	Eigen::Vector2d cc_;
	double alpha_c_;

	/// Output:
	Eigen::Vector4f bestQuat;	// best estimate quaternion
	float bestRes;	// and residuals correspond to

	void normalizeLines() {
		// // std::cout << "Normalizing lines cols: " << unnormalized_lines.cols() << std::endl;
		lines = Eigen::Matrix3Xf::Zero(3,unnormalized_lines.cols());
		for (int i = 0; i < unnormalized_lines.cols(); i++) {

			Eigen::Vector2d x1 = (unnormalized_lines.block<2,1>(0,i) - cc_).cwiseQuotient(fc_);
			Eigen::Vector2d x2 = (unnormalized_lines.block<2,1>(2,i) - cc_).cwiseQuotient(fc_);


			Eigen::Vector3d u1;
			u1 << x1, 1;
			Eigen::Vector3d u2;
			u2 << x2, 1;
			Eigen::Vector3f null = (u1.cross(u2)).cast<float>();
			lines.block<3,1>(0,i)= null/null.norm();

		}
		data_size = lines.cols();
	}

public:	
	/// Transfering between SolveMinimal and GetInliers
//	VP::Hyp* hyp;	// Hypothesis Vanishing Points
    std::vector<VP::Hyp>        hyp;
	int num_sol;	// Number of hypothesis solutions
	Eigen::Vector4f betterQuat; // For handle inside SolveMinimal()
	Eigen::MatrixXf betterGline;	 // -------------"---------------
	int betterInliersCount;
	Profiler* profiler_; // pass in profiler for vp_ransac
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/// Constructor and Destructor
	ClassifyLines();
	ClassifyLines(int _kMaxHypotheses, int _kMinHypotheses, double _kInClassResidualThreshold,
				double _kColinearityThreshold, double _kSizeRejctionPercent, double _kEarlyTerminationPercent);
	virtual ~ClassifyLines();
	
	/// Setters and Getters
	void setLines(const Eigen::MatrixXd& _lines){
		lines = _lines.cast<float>();
		data_size = _lines.cols();
	}
	
	void setUnnormalizedLines(const Eigen::MatrixXd& un_lines) {
		unnormalized_lines = un_lines;
		normalizeLines();
	}

	void setParameters(const Eigen::Vector2d& _fc, const Eigen::Vector2d& _cc, double _alpha_c) {
		fc_ = _fc;
		cc_ = _cc;
		alpha_c_ = _alpha_c;
	}

	int getMaxHypotheses(){
		return kMaxHypotheses;
	}

	Eigen::Vector4f getBestQuat(){
		return bestQuat;
	}

	float getBestRes(){
		return bestRes;
	}

	/// Operation Functions
	/// @fn SolveMinimal, Estimate the unit translation vector based on the minimal problem,
	/// given the indices of the points used in the estimation.
	void SolveMinimal(const std::vector<int>& indexes);
	/// @fn GetInliers, Classify the datapoint to inliers and outliers based on the epipolar constraint.
	void GetInliers(std::vector<int> &inlier_index, std::vector<int> &outlier_index);
	/// @fn SolveWithInliers, Estimate the unit translation vector based on the epipolar constraint,
	/// given the indices of the points used in the estimation (i.e., estimated inliers).
	void SolveWithInliers(const std::vector<int>& indexes_of_inliers);
	/// @fn ReClassify, after taking the best consensus set of G_lines and lineSubset, re-estimate to get quat.
	Eigen::MatrixXi ReClassify(const Eigen::MatrixXf& lines);

	/// @fn setProfiler
	inline void setProfiler(Profiler* prof) {
		profiler_ = prof;
	}
	/// @fn profiler
	inline Profiler* profiler() {
		return profiler_;
	}

};	/** End of class: ClassifyLines */
} 	/** End of namespace: Ransac */
#endif	/** End of file: CLASSIFY_LINES_H_*/
