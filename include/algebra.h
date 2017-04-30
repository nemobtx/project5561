#ifndef _ALGEBRA_H_
#define _ALGEBRA_H_

//#undef __ARM_NEON__

#ifdef __ARM_NEON__
#include "algebra3d.h"
#endif

#include <tbb/parallel_for.h>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Jacobi>

/** @namespace algebra
 * The algebra namespace.
 */
namespace algebra {

const float _FLOAT_DENOM_LIMIT_ = 1e-20;

#ifdef __ARM_NEON__

inline float DotProduct(Eigen::Matrix<float,3,1> const &A,
    Eigen::Matrix<float,3,1> const &B) {
  return inner3x1(A, B);
}
inline void CrossProduct(Eigen::Matrix<float,3,1> const &A,
    Eigen::Matrix<float,3,1> const &B,
    Eigen::Matrix<float,3,1> &C) {
  cross3x1(A, B, C);
}

inline void MultiplyScalar(const float a,
    Eigen::Matrix<float,3,1> const &B,
    Eigen::Matrix<float,3,1> &C) {
  mul1x1_3x1(a, B, C);
}

inline void OuterProduct(Eigen::Matrix<float,3,1> const &A,
    Eigen::Matrix<float,3,1> const &B,
    Eigen::Matrix<float,3,3> &C) {
  outer3x1(A, B, C);
}
#endif

template<typename Derived1, typename Derived2>
typename Derived1::Scalar DotProduct(Eigen::MatrixBase<Derived1> const &A,
                                     Eigen::MatrixBase<Derived2> const &B) {
  return A.dot(B);
}

template<typename Derived1, typename Derived2, typename Derived3>
inline void CrossProduct(Eigen::MatrixBase<Derived1> const &A,
                         Eigen::MatrixBase<Derived2> const &B,
                         Eigen::MatrixBase<Derived3> &C) {
  C = A.cross(B);
}

template<typename Derived1, typename Derived2>
inline Eigen::Matrix<typename Derived2::Scalar, Derived2::RowsAtCompileTime,
    Derived2::ColsAtCompileTime> CrossProduct(
    Eigen::MatrixBase<Derived1> const &A,
    Eigen::MatrixBase<Derived2> const &B) {
  Eigen::Matrix<typename Derived2::Scalar, Derived2::RowsAtCompileTime,
      Derived2::ColsAtCompileTime> C;
  C = A.cross(B);
  return C;
}

template<typename Derived1, typename Derived2, typename Derived3>
inline void MultiplyScalar(const Derived1 a,
                           Eigen::MatrixBase<Derived2> const &B,
                           Eigen::MatrixBase<Derived3> &C) {
  C = a * B;
}

template<typename Derived1, typename Derived2>
inline Eigen::Matrix<typename Derived2::Scalar, Derived2::RowsAtCompileTime,
    Derived2::ColsAtCompileTime> MultiplyScalar(
    const Derived1 a, Eigen::MatrixBase<Derived2> const &B) {
  Eigen::Matrix<typename Derived2::Scalar, Derived2::RowsAtCompileTime,
      Derived2::ColsAtCompileTime> C;
  C = a * B;
  return C;
}

template<typename Derived1, typename Derived2, typename Derived3>
inline void OuterProduct(Eigen::MatrixBase<Derived1> const &A,
                         Eigen::MatrixBase<Derived2> const &B,
                         Eigen::MatrixBase<Derived3> &C) {
  C = A * B.transpose();
}

template<typename Derived1, typename Derived2>
inline Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime,
    Derived2::RowsAtCompileTime> OuterProduct(
    Eigen::MatrixBase<Derived1> const &A,
    Eigen::MatrixBase<Derived2> const &B) {
  Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime,
      Derived2::RowsAtCompileTime> C;
  C = A * B.transpose();
  return C;
}

template<typename Derived1, typename Derived2, typename Derived3>
inline void Multiply(Eigen::MatrixBase<Derived1> const &A,
                     Eigen::MatrixBase<Derived2> const &B,
                     Eigen::MatrixBase<Derived3> &C) {
  C = A * B;
}

template<typename Derived1, typename Derived2>
inline Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime,
    Derived2::ColsAtCompileTime> Multiply(
    Eigen::MatrixBase<Derived1> const &A,
    Eigen::MatrixBase<Derived2> const &B) {
  Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime,
      Derived2::ColsAtCompileTime> C;
  C = A * B;
  return C;
}

template<typename Derived1, typename Derived2, typename Derived3>
inline void Add(Eigen::MatrixBase<Derived1> const &A,
                Eigen::MatrixBase<Derived2> const &B,
                Eigen::MatrixBase<Derived3> &C) {
  C = A + B;
}

template<typename Derived1, typename Derived2>
inline Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime,
    Derived1::ColsAtCompileTime> Add(Eigen::MatrixBase<Derived1> const &A,
                                     Eigen::MatrixBase<Derived2> const &B) {
  Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime,
      Derived1::ColsAtCompileTime> C;
  C = A + B;
  return C;
}

template<typename Derived1, typename Derived2>
inline void Normalize(Eigen::MatrixBase<Derived1> const &A,
                      Eigen::MatrixBase<Derived2> &C) {
  C = A.normalized();
}

template<typename Derived1>
inline Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime,
    Derived1::ColsAtCompileTime> Normalize(
    Eigen::MatrixBase<Derived1> const &A) {
  Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime,
      Derived1::ColsAtCompileTime> C;
  C = A.normalized();
  return C;
}

template<typename Derived1, typename Derived2>
inline void Transpose(Eigen::MatrixBase<Derived1> const &A,
                      Eigen::MatrixBase<Derived2> &C) {
  C = A.transpose();
}

template<typename Derived1>
inline Eigen::Matrix<typename Derived1::Scalar, Derived1::ColsAtCompileTime,
    Derived1::RowsAtCompileTime> Transpose(
    Eigen::MatrixBase<Derived1> const &A) {
  Eigen::Matrix<typename Derived1::Scalar, Derived1::ColsAtCompileTime,
      Derived1::RowsAtCompileTime> C;
  C = A.transpose();
  return C;
}

/// @fn ConditionNumber, Compute the 2-norm condition number of A.
template <typename Derived>
inline double ConditionNumber(const Eigen::MatrixBase<Derived> & _A) {
  /// Perform SVD decomposition on A.
   Eigen::MatrixXd A = _A.template cast<double>();
   Eigen::JacobiSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
   /// Compute the condition number.
   Eigen::VectorXd sigmas = SVD.singularValues();
   return (sigmas(sigmas.rows() - 1) != 0.0)?
       sigmas(0) / sigmas(sigmas.rows() - 1) : INFINITY;
}

/// @fn RecoverBlock, Compute a block of matrix P = _R * _R'.
template <typename Derived1, typename Derived2>
inline void RecoverBlock(const Eigen::MatrixBase<Derived1> & _R,
                         const int _i, const int _j,
                         const int _rows, const int _cols,
                         Eigen::MatrixBase<Derived2> const & _block) {
  // Cast away the const-ness of _block.
  Eigen::MatrixBase<Derived2> & block_ = const_cast<Eigen::MatrixBase<Derived2> &>(_block);
  /// Case that the requested block is square.
  if (_i == _j && _rows == _cols) {
    /// Get the upper triangular part.
    block_.template triangularView<Eigen::Upper>() =
        _R.block(_j, _j, _rows, _R.cols() - _j)
        * _R.block(_j, _j, _cols, _R.cols() - _j).template
        triangularView<Eigen::Upper>().transpose();
    /// Get the lower triangular part.
    block_.template triangularView<Eigen::StrictlyLower>() =
        block_.template triangularView<Eigen::StrictlyUpper>().transpose();
  /// Case the requested block is rectangular.
  } else {
    if (_i < _j) {
      block_.noalias() =
          _R.block(_i, _j, _rows, _R.cols() - _j)
          * _R.block(_j, _j, _cols, _R.cols() - _j).template
          triangularView<Eigen::Upper>().transpose();
    } else {
      block_.noalias() =
          _R.block(_i, _i, _rows, _R.cols() - _i).template
          triangularView<Eigen::Upper>()
          * _R.block(_j, _i, _cols, _R.cols() - _i).transpose();
    }
  }
}

/// @fn RecoverBlock, Compute a block of matrix P = _R * _R'.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic,
Eigen::AutoAlign | (Eigen::internal::traits<Derived>::Flags & Eigen::RowMajorBit ?
    Eigen::RowMajor:Eigen::ColMajor)> RecoverBlock(const Eigen::MatrixBase<Derived> & _R,
                                                   const int _i, const int _j,
                                                   const int _rows, const int _cols) {
  Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic,
    Eigen::AutoAlign | (Eigen::internal::traits<Derived>::Flags &
        Eigen::RowMajorBit ? Eigen::RowMajor:Eigen::ColMajor)> _block(_rows, _cols);
  RecoverBlock(_R, _i, _j, _rows, _cols, _block);
  return _block;
}

/// @fn ComputeGivens, Compute cos(theta), sin(theta) of the givens rotation operator.
inline bool ComputeGivens(const float _a, const float _b,
                          float & _c, float & _s) {
#define GIVENS_MATLAB
//#define GIVENS_GOLUB
//#define GIVENS_OPTIMIZED
#ifdef GIVENS_MATLAB
  if (_b == 0.0) {
    _c = 1.0; _s = 0.0;
    return false;
  } else {
    float r = std::sqrt(_a * _a + _b * _b);
    _c = _a / r;
    _s = -_b / r;
    return true;
  }
#endif
#ifdef GIVENS_GOLUB
  if (_b == 0.0) {
    _c = 1.0; _s = 0.0;
    return false;
  } else if (fabs(_b) > fabs(_a)) {
    float tau = -_a / _b;
    _s = 1.0 / std::sqrt(1.0f + tau * tau);
    _c = _s * tau;
    return true;
  } else {
    float tau = -_b / _a;
    _c = 1.0 / std::sqrt(1.0f + tau * tau);
    _s = _c * tau;
    return true;
  }
#endif
#ifdef GIVENS_OPTIMIZED
  if(_b == 0) {
    _c = 1; _s = 0;
    return false;
  }
  float tau = ((fabs(_b) > fabs(_a)))? -_a / _b : -_b / _a; // Note: might raise exception.
  _s = 1 / sqrtf(1 + tau * tau);
  _c = ((fabs(_b) > fabs(_a)))? _s * tau : _s;
  _s = ((fabs(_b) > fabs(_a)))? _s : _c * tau;
  return true;
#endif
}

/// @fn ComputeHouseholder, Compute b, u that define the Householder reflection operator.
inline bool ComputeHouseholder(Eigen::Ref<Eigen::Matrix<float,
                              Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _x,
                              Eigen::Ref<Eigen::VectorXf> _u, float & _b) {
  _u = _x;
#ifdef __ARM_NEON__
  float sigma = inner1xn(_u.data() + 1, _u.data() + 1, _u.rows() - 1);
#else
  float sigma = _u.tail(_u.rows() - 1).squaredNorm();
#endif
  if (sigma < _FLOAT_DENOM_LIMIT_) {
    _b = 0;  return false;
  }
  float x_norm = std::sqrt(_u.coeff(0)*_u.coeff(0) + sigma);
  _u(0) = (_u.coeff(0) > 0.0f)? _u.coeff(0) + x_norm : _u.coeff(0) - x_norm;
  _b = 2 / (_u.coeff(0)*_u.coeff(0) + sigma);
  return true;
}

/// @fn ApplyGivens, Apply givens rotation in a Matrix stored row-wise.
// Note: The function is optimized for big matrices (_num_cols >= 8).
inline void ApplyGivens(Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic,
                            Eigen::Dynamic, Eigen::RowMajor>> _A,
                        const int _start_index_row, const int _start_index_col,
                        const int _num_cols, const float _c, const float _s) {
#ifndef __ARM_NEON__
  // Givens rotation operator
  Eigen::JacobiRotation<float> givens_rotation(_c, _s);
  _A.block<2, Eigen::Dynamic>(_start_index_row, _start_index_col,
                             2, _num_cols).applyOnTheLeft(0, 1, givens_rotation.adjoint());
#else
  // Givens rotation operator
  Eigen::Matrix2f givens_rotation;
  givens_rotation << _c, -_s,
                     _s,  _c;
  mul2x2_2xn(givens_rotation.data(),
             _A.data() + _start_index_row * _A.innerSize() + _start_index_col,
             _A.data() + _start_index_row * _A.innerSize() + _start_index_col,
             _num_cols, _A.innerSize(), _A.innerSize());
#endif
}

/// @fn ApplyGivensThin, Apply givens rotation in a Matrix stored row-wise.
// Note: The function is optimized for thin matrices (_num_cols < 8).
inline void ApplyGivensThin(Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic,
                               Eigen::Dynamic, Eigen::RowMajor>> _A,
                             const int _start_index_row, const int _start_index_col,
                             const int _num_cols, const float _c, const float _s) {
#ifndef __ARM_NEON__
  // Givens rotation operator
  Eigen::JacobiRotation<float> givens_rotation(_c, _s);
  _A.block<2, Eigen::Dynamic>(_start_index_row, _start_index_col,
                             2, _num_cols).applyOnTheLeft(0, 1, givens_rotation.adjoint());
#else
  // Givens rotation operator
  Eigen::Matrix2f givens_rotation;
  givens_rotation << _c, -_s,
                     _s,  _c;
  mul2x2_2xn_small(givens_rotation.data(),
                   _A.data() + _start_index_row * _A.innerSize() + _start_index_col,
                   _A.data() + _start_index_row * _A.innerSize() + _start_index_col,
                   _num_cols, _A.innerSize(), _A.innerSize());
#endif
}

/// @fn ApplyGivens1, Apply givens rotation in a Vector stored column-wise.
inline void ApplyGivens1(Eigen::Ref<Eigen::VectorXf> _v, const int _start_index_row,
                         const float _c, const float _s) {
  // Get the pointer at the vector segment.
  float * v = _v.data() + _start_index_row;
  // Apply the givens rotation operator.
  float r0 = _c * (*v);
  float r1 = _s * (*v);
  r0 += (-_s) * (*(v + 1));
  r1 += _c * (*(v + 1));
  // Store the result.
  (*v) = r0;
  (*(v + 1)) = r1;
}

/// @fn ApplyHouseholder, Apply Householder reflection on a Matrix stored row-wise.
inline void ApplyHouseholder(Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic,
                            Eigen::Dynamic, Eigen::RowMajor>> _A,
                            const int _start_index_row, const int _start_index_col,
                            const int _num_cols, Eigen::Ref<Eigen::VectorXf> _u, float _b) {
#ifndef __ARM_NEON__
  _A.block(_start_index_row, _start_index_col, _u.rows(), _num_cols) -=
      _u * (_b * (_u.transpose() * _A.block(_start_index_row, _start_index_col, _u.rows(), _num_cols)));
#else
    float * uA = new float[_num_cols];
    float * _A_block = _A.data() + _start_index_row * _A.innerSize() + _start_index_col;
    mul1xn_nxm(_u.data(), _A_block, uA, _u.rows(), _num_cols, _A.innerSize());
    addmxn_mul1x1_outer_mx1_1xn(_A_block, (-_b), _u.data(), uA, _A_block,
                                _u.rows(), _num_cols, _A.innerSize(), _A.innerSize());
    delete[] uA;
#endif
}

/// @fn ApplyHouseholder1, Apply Householder reflection on a Vector stored column-wise.
inline void ApplyHouseholder1(Eigen::Ref<Eigen::VectorXf> _v, const int _start_index_row,
                             Eigen::Ref<Eigen::VectorXf> _u, float _b) {
#ifndef __ARM_NEON__
  _v.segment(_start_index_row, _u.rows()) -=
      _u * (_b * (_u.dot(_v.segment(_start_index_row, _u.rows()))));
#else
  _v.segment(_start_index_row, _u.rows()) -=
      _u * (_b * inner1xn(_u.data(), _v.data() + _start_index_row, _u.rows()));
#endif
}

/// @fn InplaceGivensQR, Compute the upper triangular factor R of the QR factorization,
/// using inplace givens rotations, without explicitly forming the matrix Q.
/// Note: The function applies for matrices stored row-wise.
inline void InplaceGivensQR(Eigen::Ref<Eigen::Matrix<float,
                     Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _A) {
  /// Number of column iterations to apply Givens.
  int num_col_its = (_A.cols() < _A.rows())? _A.cols() : _A.rows() - 1;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < num_col_its; j++) {
    /// For each row starting from the bottom of A up to j.
    for (int i = _A.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(_A(i, j), _A(i + 1, j), c, s);
      algebra::ApplyGivens(_A, i, j, _A.cols() - j, c, s);
    }
  }
}

/// @fn InplaceGivensQR, Compute the upper triangular factor R of the QR factorization,
/// using inplace givens rotations, without explicitly forming the matrix Q.
/// Note: The function applies for matrices stored row-wise.
inline void InplaceGivensQRWithVectorMatrix(Eigen::Ref<Eigen::Matrix<float,
        Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _A,
        Eigen::Ref<Eigen::VectorXf> _b,
        Eigen::Ref<Eigen::Matrix<float,
        Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _H) {
    /// Number of column iterations to apply Givens.
    int num_col_its = (_A.cols() < _A.rows()) ? _A.cols() : _A.rows() - 1;
    /// For each column of the matrix A that is to be triangularized.
    for (int j = 0; j < num_col_its; j++) {
        /// For each row starting from the bottom of A up to j.
        for (int i = _A.rows() - 2; i >= j; i--) {
            float c, s;
            /// Compute the givens rotation operator and apply it accordingly.
            algebra::ComputeGivens(_A(i, j), _A(i + 1, j), c, s);
            algebra::ApplyGivens(_A, i, j, _A.cols() - j, c, s);
            algebra::ApplyGivens(_H, i, 0, _H.cols(), c, s);
            algebra::ApplyGivens1(_b, i,  c, s);
        }
    }
}


/// @fn ThinInplaceGivensQR, Compute the upper triangular factor R of the QR factorization,
/// using inplace givens rotations, without explicitly forming the matrix Q.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function is optimized for matrices with small number of columns (n < 8).
inline void ThinInplaceGivensQR(Eigen::Ref<Eigen::Matrix<float,
                     Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _A) {
  /// Number of column iterations to apply Givens.
  int num_col_its = (_A.cols() < _A.rows())? _A.cols() : _A.rows() - 1;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < num_col_its; j++) {
    /// For each row starting from the bottom of A up to j.
    for (int i = _A.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(_A(i, j), _A(i + 1, j), c, s);
      algebra::ApplyGivensThin(_A, i, j, _A.cols() - j, c, s);
    }
  }
}

/// @fn ConditionalInplaceGivensQR, Compute the upper triangular factor R of the QR factorization,
/// using conditional inplace givens rotations, without explicitly forming the matrix Q.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function takes advantage of any sparsity in the bottom left corner.
inline void ConditionalInplaceGivensQR(Eigen::Ref<Eigen::Matrix<float,
                     Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _A) {
  /// Number of column iterations to apply Givens.
  int num_col_its = (_A.cols() < _A.rows())? _A.cols() : _A.rows() - 1;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < num_col_its; j++) {
    /// For each row starting from the bottom of A up to j.
    for (int i = _A.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      if (algebra::ComputeGivens(_A(i, j), _A(i + 1, j), c, s)) {
        algebra::ApplyGivens(_A, i, j, _A.cols() - j, c, s);
      }
    }
  }
}

struct GivensJob {
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> * A_;
  int start_row_index_;
  int start_col_index_;
  int cols_;
  int final_row_index_;
  GivensJob(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> * _A,
            int _i, int  _j, int _cols, int _final_row_index)
  : A_ (_A),
    start_row_index_(_i),
    start_col_index_(_j),
    cols_(_cols),
    final_row_index_(_final_row_index) {}
  inline void Do() {
    float c,s;
    algebra::ComputeGivens((*A_)(start_row_index_, start_col_index_),
                           (*A_)(start_row_index_ + 1, start_col_index_), c, s);
    algebra::ApplyGivens((*A_), start_row_index_, start_col_index_, cols_, c, s);
    start_row_index_--;
  }
};

struct ParallelGivens {
  std::deque<GivensJob> * job_;
  ParallelGivens(std::deque<GivensJob> * _job)
  : job_(_job){}
  void operator() (const tbb::blocked_range<int> & _r) const {
    for ( int i = _r.begin(); i != _r.end(); i++){
      (*job_)[i].Do();
    }
  }
};

inline void ParallelInplaceGivensQR(Eigen::Matrix<float, Eigen::Dynamic,
                                    Eigen::Dynamic, Eigen::RowMajor> & A) {
#define DELAY 2
#define GRAIN_SIZE 5
#define DENOM_IN_SHIFTS 2
  std::deque<GivensJob> jobs;
  int j = 0;
  int i = A.rows() - 2;

  // Part of the matrix where jobs are both created and executed.
  for (; i >= (A.rows() - 2) - (A.cols() - 1) * DELAY; i--) {
    // Create a new job.
    if ( ((A.rows() - 2) - i) % 2 == 0 ) {
      jobs.emplace_back(&A, A.rows() - 2, j, A.cols() - j, j);
      j++;
    }
    // Do available jobs.
    tbb::parallel_for(tbb::blocked_range<int>(0, jobs.size(), GRAIN_SIZE),
                      ParallelGivens(&jobs));
    // Clean previous jobs.
    if (jobs[0].start_row_index_ < jobs[0].final_row_index_) {
      jobs.pop_front();
    }
  }

  // Part of the matrix where jobs are executed.
  while (!jobs.empty()) {
    // Do available jobs.
    tbb::parallel_for(tbb::blocked_range<int>(0, jobs.size(), GRAIN_SIZE),
                      ParallelGivens(&jobs));
    // Clean previous jobs.
    if (jobs[0].start_row_index_ < jobs[0].final_row_index_) {
      jobs.pop_front();
    }
  }
}

/// @fn InplaceHouseholderQR, Compute the upper triangular factor R of the QR factorization,
/// using inplace Householder reflections, without explicitly forming the matrix Q.
/// Note: The function applies for matrices stored row-wise.
inline void InplaceHouseholderQR(Eigen::Ref<Eigen::Matrix<float,
                     Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _A) {
  Eigen::VectorXf u(_A.rows());
  float b;
  /// Number of column iterations to apply Householder.
  int num_col_its = (_A.cols() < _A.rows())? _A.cols() : _A.rows() - 1;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < num_col_its; j++) {
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(_A.block(j, j, _A.rows() - j, 1), u.head(_A.rows() - j), b);
    algebra::ApplyHouseholder(_A, j, j, _A.cols() - j, u.head(_A.rows() - j), b);
  }
}


/// @fn InplaceHousholderQR, Compute the upper triangular factor R of the QR factorization,
/// using inplace housholder reflections, without explicitly forming the matrix Q.
/// Note: The function applies for matrices stored row-wise.
inline void ConditionalInplaceHouseholderQRWithVector(Eigen::Ref<Eigen::Matrix<float,
Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _A,
Eigen::Ref<Eigen::Matrix<float,Eigen::Dynamic, 1>> _b) {
  Eigen::VectorXf u(_A.rows());
  float b;
  /// Number of column iterations to apply Householder.
  int num_col_its = (_A.cols() < _A.rows())? _A.cols() : _A.rows() - 1;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < num_col_its; j++) {
    /// Compute the Housholder reflection operator and apply it accordingly.
    if (algebra::ComputeHouseholder(_A.block(j, j, _A.rows() - j, 1), u.head(_A.rows() - j), b)) {
      algebra::ApplyHouseholder(_A, j, j, _A.cols() - j, u.head(_A.rows() - j), b);
      algebra::ApplyHouseholder1(_b, j, u.head(_A.rows() - j), b);
    }
  }
}

/// @fn InplaceHousholderQR, Compute the upper triangular factor R of the QR factorization,
/// using inplace housholder reflections, without explicitly forming the matrix Q.
/// Note: The function applies for matrices stored row-wise.
inline void ConditionalInplaceHouseholderQRWithVectorMatrix(Eigen::Ref<Eigen::Matrix<float,
Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _A,
Eigen::Ref<Eigen::Matrix<float,Eigen::Dynamic, 1>> _b,
Eigen::Ref<Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > c_m) {
  Eigen::VectorXf u(_A.rows());
  float b;
  /// Number of column iterations to apply Householder.
  int num_col_its = (_A.cols() < _A.rows())? _A.cols() : _A.rows() - 1;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < num_col_its; j++) {
    /// Compute the Housholder reflection operator and apply it accordingly.
    if (algebra::ComputeHouseholder(_A.block(j, j, _A.rows() - j, 1), u.head(_A.rows() - j), b)) {
      algebra::ApplyHouseholder(_A, j, j, _A.cols() - j, u.head(_A.rows() - j), b);
      algebra::ApplyHouseholder(c_m, j, 0, c_m.cols(), u.head(_A.rows()-j), b);
      algebra::ApplyHouseholder1(_b, j, u.head(_A.rows() - j), b);
    }
  }
}

/// @fn InplaceHouseholderQR, Compute the upper triangular factor R of the QR factorization,
/// using inplace Householder reflections, without explicitly forming the matrix Q.
/// Note: The function applies for matrices stored row-wise.
inline void ConditionalInplaceHouseholderQR(Eigen::Ref<Eigen::Matrix<float,
                     Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _A) {
  Eigen::VectorXf u(_A.rows());
  float b;
  /// Number of column iterations to apply Householder.
  int num_col_its = (_A.cols() < _A.rows())? _A.cols() : _A.rows() - 1;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < num_col_its; j++) {
    /// Compute the Householder reflection operator and apply it accordingly.
    if (algebra::ComputeHouseholder(_A.block(j, j, _A.rows() - j, 1), u.head(_A.rows() - j), b)) {
      algebra::ApplyHouseholder(_A, j, j, _A.cols() - j, u.head(_A.rows() - j), b);
    }
  }
}

/// @fn LeastSquaresSolve3, Solve the mx3 Least Squares Problem.
/// Note: The right hand side should be appended on the matrix A ([A,b]).
/// Note: The function applies for matrices stored row-wise.
inline bool LeastSquaresSolve3(Eigen::Ref<Eigen::Matrix<float,
                     Eigen::Dynamic, 4, Eigen::RowMajor>> _A,
                     Eigen::Ref<Eigen::Vector3f> _x,
                     float _threshold) {
#define LS_USE_GIVENS
#ifdef LS_USE_HOUSEHOLDER
  Eigen::VectorXf u(_A.rows());
  float b;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < 3; j++) {
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(_A.block(j, j, _A.rows() - j, 1), u.head(_A.rows() - j), b);
    algebra::ApplyHouseholder(_A, j, j, _A.cols() - j, u.head(_A.rows() - j), b);
  }
#endif
#ifdef LS_USE_GIVENS
  // For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < 3; j++) {
    // For each row starting from the bottom of A up to j.
    for (int i = _A.rows() - 2; i >= j; i--) {
      float c, s;
      // Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(_A(i, j), _A(i + 1, j), c, s);
      algebra::ApplyGivens(_A, i, j, _A.cols() - j, c, s);
    }
  }
#endif
  // Perform the Approximate condition number check.
  Eigen::Vector3f R_diagonal_abs = _A.block<3, 3>(0, 0).diagonal().cwiseAbs();
  float condition_number = (R_diagonal_abs.minCoeff() != 0)?
        R_diagonal_abs.maxCoeff()/R_diagonal_abs.minCoeff() : INFINITY;
  if (condition_number > _threshold) {
    return false;
  }
  // Perform the 3x3 back-solve.
  _A.block<3, 3>(0, 0).triangularView<Eigen::StrictlyLower>().setZero();
  _x = _A.block<3, 3>(0, 0).triangularView<Eigen::Upper>().
      solve(_A.block<3, 1>(0, 3));
  return true;
}

/// @fn LNSTransformation, Transform the measurement model equation
/// z = Hf*xf + Hr*xr + n, to Q'*z = Q'*Hf*xf + Q'*Hr*xr + Q'*n in an inplace fashion.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function supports constant velocity measurement model,
/// interpolation measurement model, inverse depth parameterization.
inline void LNStransformation(Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                 Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                 Eigen::Ref<Eigen::VectorXf> _z,
                 int _cols_pose, int _cols_dense_part,
                 int _zero_cols_front = 0, int _zero_cols_back = 0) {
  /// Compute the maximum number of columns that the ladder part can take.
  int max_cols_ladder_part = _Hr.cols() - (_zero_cols_front + _zero_cols_back + _cols_dense_part);
  /// For each column of the matrix Hf.
  for (int j = 0; j < _Hf.cols(); j++) {
    /// For each row starting from the bottom of Hf up to j.
    for (int i = _Hf.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator.
      algebra::ComputeGivens(_Hf(i, j), _Hf(i + 1, j), c, s);
      /// Apply the givens operator on Hf.
      algebra::ApplyGivensThin(_Hf, i, j, _Hf.cols() - j, c, s);
      /// Apply the givens operator on z.
      algebra::ApplyGivens1(_z, i, c, s);

      /// Offset to how many times cols_pose the givens operator will be applied.
      int offset = ((j + 1) >> 1) + 2;
      /// Step to how many times cols_pose the givens operator will be applied.
      int step = (((_Hf.rows() - 2) - i) + ((j + 1) % 2)) >> 1;
      /// Number of columns that givens operator will be applied.
      int num_cols = ((offset + step) * _cols_pose < max_cols_ladder_part)?
          (offset + step) * _cols_pose : max_cols_ladder_part;
      /// Starting row index on Hr that the givens operator will be applied.
      int start_index_row = i;
      /// Starting column index on Hr that the givens operator will be applied.
      int start_index_col = _Hr.cols() - (_cols_dense_part + _zero_cols_back + num_cols);
      /// Apply the givens operator on Hr.
      algebra::ApplyGivens(_Hr, start_index_row, start_index_col, num_cols, c, s);
      /// Apply the givens operator on the dense part at the end of Hr.
      algebra::ApplyGivens(_Hr, start_index_row, _Hr.cols() - _cols_dense_part, _cols_dense_part, c, s);

//      std::cout << "Hf:\t" << "(" << i << "," << j << "," << _Hf.cols() - j << ")" << std::endl;
//      std::cout << "z:\t"<< "(" << i << "," << 0 << "," << 1 << ")" << std::endl;
//      std::cout << "offset: " << offset << std::endl;
//      std::cout << "step: " << step << std::endl;
//      std::cout << "Hr:\t"<< "(" << start_index_row << "," << start_index_col << "," << num_cols << ")" << std::endl;
    }
  }
}

/// @fn LNSTransformation, Transform the measurement model equation
/// z = Hf*xf + Hr*xr + n, to Q'*z = Q'*Hf*xf + Q'*Hr*xr + Q'*n in an inplace fashion.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function supports constant velocity measurement model,
/// interpolation measurement model, inverse depth parameterization.
inline void LNStransformationMultistep(Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                 Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                 Eigen::Ref<Eigen::VectorXf> _z,
                 int _cols_pose, int _cols_dense_part,
                 int _zero_cols_front = 0, int _zero_cols_back = 0) {
  /// Compute the maximum number of columns that the ladder part can take.
  int max_cols_ladder_part = _Hr.cols() - (_zero_cols_front + _zero_cols_back + _cols_dense_part);
  /// For each column of the matrix Hf.
  for (int j = 0; j < _Hf.cols(); j++) {
    /// For each row starting from the bottom of Hf up to j.
    for (int i = _Hf.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator.
      algebra::ComputeGivens(_Hf(i, j), _Hf(i + 1, j), c, s);
      /// Apply the givens operator on Hf.
      algebra::ApplyGivensThin(_Hf, i, j, _Hf.cols() - j, c, s);
      /// Apply the givens operator on z.
      algebra::ApplyGivens1(_z, i, c, s);

      /// Offset to how many times cols_pose the givens operator will be applied.
      int offset = ((j + 1) >> 1) + 2 + (max_cols_ladder_part/_cols_pose - _Hf.rows()/2);
      /// Step to how many times cols_pose the givens operator will be applied.
      int step = (((_Hf.rows() - 2) - i) + ((j + 1) % 2)) >> 1;
      /// Number of columns that givens operator will be applied.
      int num_cols = ((offset + step) * _cols_pose < max_cols_ladder_part)?
          (offset + step) * _cols_pose : max_cols_ladder_part;
      /// Starting row index on Hr that the givens operator will be applied.
      int start_index_row = i;
      /// Starting column index on Hr that the givens operator will be applied.
      int start_index_col = _Hr.cols() - (_cols_dense_part + _zero_cols_back + num_cols);
      /// Apply the givens operator on Hr.
      algebra::ApplyGivens(_Hr, start_index_row, start_index_col, num_cols, c, s);
      /// Apply the givens operator on the dense part at the end of Hr.
      algebra::ApplyGivens(_Hr, start_index_row, _Hr.cols() - _cols_dense_part, _cols_dense_part, c, s);

//      std::cout << "Hf:\t" << "(" << i << "," << j << "," << _Hf.cols() - j << ")" << std::endl;
//      std::cout << "z:\t"<< "(" << i << "," << 0 << "," << 1 << ")" << std::endl;
//      std::cout << "offset: " << offset << std::endl;
//      std::cout << "step: " << step << std::endl;
//      std::cout << "Hr:\t"<< "(" << start_index_row << "," << start_index_col << "," << num_cols << ")" << std::endl;
    }
  }
}

/// @fn LNSTransformation, Transform the measurement model equation
/// z = Hf*xf + Hr*xr + n, to Q'*z = Q'*Hf*xf + Q'*Hr*xr + Q'*n in an inplace fashion.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function supports constant velocity measurement model,
/// interpolation measurement model, inverse depth parameterization.
/// _corners: The position of corners (from the bottom to top).
inline void LNStransformationWithCorners(Eigen::Ref<Eigen::Matrix<float,
                  Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                  Eigen::Ref<Eigen::Matrix<float,
                  Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                  Eigen::Ref<Eigen::VectorXf> _z,
                  std::vector<std::pair<int, int>> &_corners,
                  int _cols_dense_part, int _zero_cols_front = 0,
                  int _zero_cols_back = 0) {
  /// Compute the maximum number of columns that the ladder part can take.
  int max_cols_ladder_part = _Hr.cols() - (_zero_cols_front + _zero_cols_back + _cols_dense_part);
  /// For each column of the matrix Hf.
  for (int j = 0; j < _Hf.cols(); j++) {
    /// Starting from the corner on the bottom
    int corner_index = _corners.size() - 1;
    for (int k = _corners.size() - 2; k >= 0; --k) {
      if (_corners[k].first == _corners[_corners.size() - 1].first) {
        corner_index = k;
      }
    }
    /// For each row starting from the bottom of Hf up to j.
    for (int i = _Hf.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator.
      algebra::ComputeGivens(_Hf(i, j), _Hf(i + 1, j), c, s);
      /// Apply the givens operator on Hf.
      algebra::ApplyGivensThin(_Hf, i, j, _Hf.cols() - j, c, s);
      /// Apply the givens operator on z.
      algebra::ApplyGivens1(_z, i, c, s);
      /// Go to the next corner
      if (corner_index > 0 && i == _corners[corner_index - 1].first) {
        --corner_index;
      }
      int num_cols = max_cols_ladder_part + _zero_cols_front - _corners[corner_index].second;
      /// Starting row index on Hr that the givens operator will be applied.
      int start_index_row = i;
      /// Starting column index on Hr that the givens operator will be applied.
      int start_index_col = _corners[corner_index].second;
      /// Apply the givens operator on Hr.
      algebra::ApplyGivens(_Hr, start_index_row, start_index_col, num_cols, c, s);
      /// Apply the givens operator on the dense part at the end of Hr.
      algebra::ApplyGivens(_Hr, start_index_row, _Hr.cols() - _cols_dense_part, _cols_dense_part, c, s);

//      std::cout << "Hf:\t" << "(" << i << "," << j << "," << _Hf.cols() - j << ")" << std::endl;
//      std::cout << "z:\t"<< "(" << i << "," << 0 << "," << 1 << ")" << std::endl;
//      std::cout << "offset: " << offset << std::endl;
//      std::cout << "step: " << step << std::endl;
//      std::cout << "Hr:\t"<< "(" << start_index_row << "," << start_index_col << "," << num_cols << ")" << std::endl;
    }
    /// Update the positions of the corners
    for (int k = _corners.size() - 2; k >= 0; --k) {
      if (_corners[k].first < _corners[_corners.size() - 1].first) {
        ++_corners[k].first;
      }
    }
  }
  /// Remove unnecessary corners
  while (_corners.size() > 1 && _corners[_corners.size() - 1].first ==
                                   _corners[_corners.size() - 2].first) {
    _corners.pop_back();
  }
}

/// @fn LNStransformationDense, Transform the measurement model equation
/// z = Hf*xf + Hr*xr + n, to Q'*z = Q'*Hf*xf + Q'*Hr*xr + Q'*n in an inplace fashion.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function supports constant velocity measurement model,
/// interpolation measurement model, inverse depth parameterization.
inline void LNStransformationDense(Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                 Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                 Eigen::Ref<Eigen::VectorXf> _z,
                 int _cols_dense_part,
                 int _zero_cols_front = 0, int _zero_cols_back = 0) {
  /// Compute the maximum number of columns that the ladder part can take.
  int max_cols_ladder_part = _Hr.cols() - (_zero_cols_front + _zero_cols_back + _cols_dense_part);
  /// Compute the starting column index of the dense part.
  int start_index_col_dense = _Hr.cols() - _cols_dense_part;
  /// For each column of the matrix Hf.
  for (int j = 0; j < _Hf.cols(); j++) {
    /// For each row starting from the bottom of Hf up to j.
    for (int i = _Hf.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator.
      algebra::ComputeGivens(_Hf(i, j), _Hf(i + 1, j), c, s);
      /// Apply the givens operator on Hf.
      algebra::ApplyGivensThin(_Hf, i, j, _Hf.cols() - j, c, s);
      /// Apply the givens operator on z.
      algebra::ApplyGivens1(_z, i, c, s);

      /// Apply the givens operator on Hr.
      algebra::ApplyGivens(_Hr, i, _zero_cols_front, max_cols_ladder_part, c, s);
      /// Apply the givens operator on the dense part at the end of Hr.
      algebra::ApplyGivens(_Hr, i, start_index_col_dense, _cols_dense_part, c, s);
    }
  }
}

/// @fn LNSTransformationInverseDepth, Transform the measurement model equation
/// z = Hf*xf + Hr*xr + n, to Q'*z = Q'*Hf*xf + Q'*Hr*xr + Q'*n in an inplace fashion.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function supports constant velocity measurement model,
/// interpolation measurement model, inverse depth parameterization.
inline void LNStransformationInverseDepth(Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                 Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                 Eigen::Ref<Eigen::VectorXf> _z,
                 int _cols_pose, int _cols_dense_part,
                 int _zero_cols_front = 0, int _zero_cols_back = 0) {
  /// Compute the maximum number of columns that the ladder part can take.
  int max_cols_ladder_part = _Hr.cols() - (_zero_cols_front + _zero_cols_back +
                                           _cols_dense_part + _cols_pose);
  /// For each column of the matrix Hf.
  for (int j = 0; j < _Hf.cols(); j++) {
    /// For each row starting from the bottom of Hf up to j.
    for (int i = _Hf.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator.
      algebra::ComputeGivens(_Hf(i, j), _Hf(i + 1, j), c, s);
      /// Apply the givens operator on Hf.
      algebra::ApplyGivensThin(_Hf, i, j, _Hf.cols() - j, c, s);
      /// Apply the givens operator on z.
      algebra::ApplyGivens1(_z, i, c, s);

      /// Offset to how many times cols_pose the givens operator will be applied.
      int offset = ((j + 1) >> 1) + 2;
      /// Step to how many times cols_pose the givens operator will be applied.
      int step = (((_Hf.rows() - 2) - i) + ((j + 1) % 2)) >> 1;
      /// Number of columns that givens operator will be applied.
      int num_cols = ((offset + step) * _cols_pose < max_cols_ladder_part)?
          (offset + step) * _cols_pose : max_cols_ladder_part;
      /// Starting row index on Hr that the givens operator will be applied.
      int start_index_row = i;
      /// Starting column index on Hr that the givens operator will be applied.
      int start_index_col = _Hr.cols() - (_cols_dense_part + _zero_cols_back + num_cols);
      /// Apply the givens operator on Hr.
      algebra::ApplyGivens(_Hr, start_index_row, start_index_col, num_cols, c, s);
      /// Apply the givens operator on the dense part at the end of Hr.
      algebra::ApplyGivens(_Hr, start_index_row, _Hr.cols() - _cols_dense_part, _cols_dense_part, c, s);
      algebra::ApplyGivens(_Hr, start_index_row, _zero_cols_front, _cols_pose, c, s);

//      std::cout << "Hf:\t" << "(" << i << "," << j << "," << _Hf.cols() - j << ")" << std::endl;
//      std::cout << "z:\t"<< "(" << i << "," << 0 << "," << 1 << ")" << std::endl;
//      std::cout << "offset: " << offset << std::endl;
//      std::cout << "step: " << step << std::endl;
//      std::cout << "Hr:\t"<< "(" << start_index_row << "," << start_index_col << "," << num_cols << ")" << std::endl;
    }
  }
}

/// @fn LNSTransformationInverseDepth, Transform the measurement model equation
/// z = Hf*xf + Hr*xr + n, to Q'*z = Q'*Hf*xf + Q'*Hr*xr + Q'*n in an inplace fashion.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function supports constant velocity measurement model,
/// interpolation measurement model, inverse depth parameterization.
inline void LNStransformationInverseDepthMultistep(Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                 Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                 Eigen::Ref<Eigen::VectorXf> _z,
                 int _cols_pose, int _cols_dense_part,
                 int _zero_cols_front = 0, int _zero_cols_back = 0) {
  /// Compute the maximum number of columns that the ladder part can take.
  int max_cols_ladder_part = _Hr.cols() - (_zero_cols_front + _zero_cols_back +
                                           _cols_dense_part + _cols_pose);
  /// For each column of the matrix Hf.
  for (int j = 0; j < _Hf.cols(); j++) {
    /// For each row starting from the bottom of Hf up to j.
    for (int i = _Hf.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator.
      algebra::ComputeGivens(_Hf(i, j), _Hf(i + 1, j), c, s);
      /// Apply the givens operator on Hf.
      algebra::ApplyGivensThin(_Hf, i, j, _Hf.cols() - j, c, s);
      /// Apply the givens operator on z.
      algebra::ApplyGivens1(_z, i, c, s);

      /// Offset to how many times cols_pose the givens operator will be applied.
      int offset = ((j + 1) >> 1) + 2 + (max_cols_ladder_part/_cols_pose - _Hf.rows()/2);
      /// Step to how many times cols_pose the givens operator will be applied.
      int step = (((_Hf.rows() - 2) - i) + ((j + 1) % 2)) >> 1;
      /// Number of columns that givens operator will be applied.
      int num_cols = ((offset + step) * _cols_pose < max_cols_ladder_part)?
          (offset + step) * _cols_pose : max_cols_ladder_part;
      /// Starting row index on Hr that the givens operator will be applied.
      int start_index_row = i;
      /// Starting column index on Hr that the givens operator will be applied.
      int start_index_col = _Hr.cols() - (_cols_dense_part + _zero_cols_back + num_cols);
      /// Apply the givens operator on Hr.
      algebra::ApplyGivens(_Hr, start_index_row, start_index_col, num_cols, c, s);
      /// Apply the givens operator on the dense part at the end of Hr.
      algebra::ApplyGivens(_Hr, start_index_row, _Hr.cols() - _cols_dense_part, _cols_dense_part, c, s);
      algebra::ApplyGivens(_Hr, start_index_row, _zero_cols_front, _cols_pose, c, s);

//      std::cout << "Hf:\t" << "(" << i << "," << j << "," << _Hf.cols() - j << ")" << std::endl;
//      std::cout << "z:\t"<< "(" << i << "," << 0 << "," << 1 << ")" << std::endl;
//      std::cout << "offset: " << offset << std::endl;
//      std::cout << "step: " << step << std::endl;
//      std::cout << "Hr:\t"<< "(" << start_index_row << "," << start_index_col << "," << num_cols << ")" << std::endl;
    }
  }
}

/// @fn LNStransformationDenseHouseholder, Transform the measurement model equation
/// z = Hf*xf + Hr*xr + n, to Q'*z = Q'*Hf*xf + Q'*Hr*xr + Q'*n in an inplace fashion.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function supports constant velocity measurement model,
/// interpolation measurement model, inverse depth parameterization.
inline void LNStransformationDenseHouseholder(Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                 Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                 Eigen::Ref<Eigen::VectorXf> _z,
                 int _cols_dense_part,
                 int _zero_cols_front = 0, int _zero_cols_back = 0) {
  /// Compute the maximum number of columns that the ladder part can take.
  int max_cols_ladder_part = _Hr.cols() - (_zero_cols_front + _zero_cols_back + _cols_dense_part);
  /// Compute the starting column index of the dense part.
  int start_index_col_dense = _Hr.cols() - _cols_dense_part;
  Eigen::VectorXf u(_Hf.rows());
  float b;
  /// For each column of the matrix A that is to be triangularized.
  for (int j = 0; j < _Hf.cols(); j++) {
    int row_size = _Hf.rows() - j;
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(_Hf.block(j, j, row_size, 1), u.head(row_size), b);
    algebra::ApplyHouseholder(_Hf, j, j, _Hf.cols() - j, u.head(row_size), b);
    algebra::ApplyHouseholder(_Hr, j, _zero_cols_front, max_cols_ladder_part, u.head(row_size), b);
    algebra::ApplyHouseholder(_Hr, j, start_index_col_dense, _cols_dense_part, u.head(row_size), b);
    algebra::ApplyHouseholder1(_z, j, u.head(row_size), b);
  }
}


/// @fn LNSTransformationEigenQR, Transform the measurement model equation
/// z = Hf*xf + Hr*xr + n, to Q'*z = Q'*Hf*xf + Q'*Hr*xr + Q'*n in an inplace fashion.
/// Note: The function applies for matrices stored row-wise.
/// Note: The function supports constant velocity measurement model,
/// interpolation measurement model, inverse depth parameterization.
inline void LNStransformationEigenQR(Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                 Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                 Eigen::Ref<Eigen::VectorXf> _z) {
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixRXf;
  Eigen::HouseholderQR<MatrixRXf> qr(_Hf);
  _Hf = qr.matrixQR();
  _Hr = qr.householderQ().transpose() * _Hr;
  _z = qr.householderQ().transpose() * _z;
}

inline void LNStransformationEigenQR_Stereo(Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hf,
                 Eigen::Ref<Eigen::Matrix<float,
                 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _Hr,
                 Eigen::Ref<Eigen::VectorXf> _z,
				 int offset, int feat_size) {
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixRXf;
  if(offset < 0) {
	  Eigen::HouseholderQR<MatrixRXf> qr(_Hf);
	  _Hf = qr.matrixQR();
	  _Hr = qr.householderQ().transpose() * _Hr;
	  _z = qr.householderQ().transpose() * _z;

  } else {
	  Eigen::HouseholderQR<MatrixRXf> qr1(_Hf.block(0, 0, offset, _Hf.cols()));
	  _Hf.block(0, 0, offset, _Hf.cols()) = qr1.matrixQR();
	  _Hr.block(0, 0, offset, _Hr.cols()) = qr1.householderQ().transpose() * _Hr.block(0, 0, offset, _Hr.cols());
	  _z.head(offset) = qr1.householderQ().transpose() * _z.head(offset);

	  Eigen::HouseholderQR<MatrixRXf> qr2(_Hf.block(offset, 0, _Hf.rows() - offset, _Hf.cols()));
	  _Hf.block(offset, 0, _Hf.rows() - offset, _Hf.cols()) = qr2.matrixQR();
	  _Hr.block(offset, 0, _Hr.rows() - offset, _Hr.cols()) = qr2.householderQ().transpose() * _Hr.block(offset, 0, _Hr.rows() - offset, _Hr.cols());
	  _z.tail(_z.size() - offset) = qr2.householderQ().transpose() * _z.tail(_z.size() - offset);

	  Eigen::MatrixXf _mini_Hf(feat_size * 2, _Hf.cols());
	  Eigen::MatrixXf _mini_Hr(feat_size * 2, _Hr.cols());
	  Eigen::VectorXf _mini_z(feat_size * 2);

	  _mini_Hf.block(0, 0, feat_size, _mini_Hf.cols()) = _Hf.block(0, 0, feat_size, _Hf.cols());
	  _mini_Hf.block(feat_size, 0, feat_size, _mini_Hf.cols()) = _Hf.block(offset, 0, feat_size, _Hf.cols());
	  _mini_Hr.block(0, 0, feat_size, _mini_Hr.cols()) = _Hr.block(0, 0, feat_size, _Hr.cols());
	  _mini_Hr.block(feat_size, 0, feat_size, _mini_Hr.cols()) = _Hr.block(offset, 0, feat_size, _Hr.cols());
	  _mini_z.head(feat_size) = _z.head(feat_size);
	  _mini_z.tail(feat_size) = _z.segment(offset, feat_size);

	  Eigen::HouseholderQR<MatrixRXf> qr_mini(_mini_Hf);
	  _mini_Hf = qr_mini.matrixQR();
	  _mini_Hr = qr_mini.householderQ().transpose() * _mini_Hr;
	  _mini_z = qr_mini.householderQ().transpose() * _mini_z;

	  _Hf.block(0, 0, feat_size, _Hf.cols()) = _mini_Hf.block(0, 0, feat_size, _mini_Hf.cols());
	  _Hf.block(offset, 0, feat_size, _Hf.cols()) = _mini_Hf.block(feat_size, 0, feat_size, _mini_Hf.cols());
	  _Hr.block(0, 0, feat_size, _Hr.cols()) = _mini_Hr.block(0, 0, feat_size, _mini_Hr.cols());
	  _Hr.block(offset, 0, feat_size, _Hr.cols()) = _mini_Hr.block(feat_size, 0, feat_size, _mini_Hr.cols());
	  _z.head(feat_size) = _mini_z.head(feat_size);
	  _z.segment(offset, feat_size) = _mini_z.tail(feat_size);
  }
}

/// @fn MeasurmentCompressionTransformation, Transform the measurement model equation
/// z = Hr*xr + n to Q'*z = R*xr + Q'*n, to reduce the row-permuted ladder-shaped Hr matrix
/// to an upper triangular factor R by applying consecutive inplace Householder reflections.
/// The function takes advantage of the structure as defined by the edge points of ladder part,
/// the zero columns in front and in the back of the ladder part.
/// Note: The function applies for a matrix stored row-wise.
// _cols_dense_part = number of dense columns in last cols of H
// _zero_cols_back = number of columns in H that are zero after ladder portion but before dense
// H = ---------
//     |xxxxx x|
//     | xxxx x|
//     |  xxx x|
//     |   xx x|
//     |    x x|
//     ---------
inline void MeasurmentCompressionTransformation(Eigen::Ref<Eigen::Matrix<float,
                           Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _H,
                           Eigen::Ref<Eigen::VectorXf> _z,
                           std::vector<std::pair<int, int>> & _index_pair,
                           int _cols_dense_part, int _zero_cols_back = 0) {
  // Useful sizes and householder utilities.
  int zero_cols_front = _index_pair[0].second;
  int cols_ladder_part = _H.cols() - (zero_cols_front + _zero_cols_back + _cols_dense_part);
  int start_col_index_dense_part = _H.cols() - _cols_dense_part;
  Eigen::VectorXf u(_H.rows());
  float b;
  // for each ladder step, except the last one, reduce its columns to upper triangular.
  for (unsigned int k = 0; k < _index_pair.size() - 1; k++) {
    // Get the column that terminates the for-loop for the reduction of this ladder step.
    int end_col = std::min(_index_pair[k + 1].second, _index_pair[k].first + zero_cols_front);
    // Reduce the jth column to upper triangular.
    for (int j = _index_pair[k].second; j < end_col; j++) {
      int row_index = j - zero_cols_front;
      int col_index = j;
      int num_rows = _index_pair[k].first + 1 - row_index;
      int num_cols = cols_ladder_part - row_index;
      /// Compute the Householder reflection operator and apply it accordingly.
      algebra::ComputeHouseholder(_H.block(row_index, col_index, num_rows, 1), u.head(num_rows), b);
      algebra::ApplyHouseholder(_H, row_index, col_index, num_cols, u.head(num_rows), b);
      algebra::ApplyHouseholder(_H, row_index, start_col_index_dense_part, _cols_dense_part, u.head(num_rows), b);
      algebra::ApplyHouseholder1(_z, row_index, u.head(num_rows), b);
//      std::cout << "(" << row_index << "," << col_index << ","
//                << num_rows << "," << num_cols << ")" << std::endl;
    }
  }
  // For the last ladder step reduce its columns to upper triangular.
  // Get the column that terminates the for-loop for the reduction of this ladder step.
  int end_col = std::min(zero_cols_front + cols_ladder_part,
                         _index_pair[_index_pair.size() - 1].first + zero_cols_front);
  // Reduce the jth column to upper triangular.
  for (int j = _index_pair[_index_pair.size() - 1].second; j < end_col; j++) {
    int row_index = j - zero_cols_front;
    int col_index = j;
    int num_rows = _index_pair[_index_pair.size() - 1].first + 1 - row_index;
    int num_cols = cols_ladder_part - row_index;
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(_H.block(row_index, col_index, num_rows, 1), u.head(num_rows), b);
    algebra::ApplyHouseholder(_H, row_index, col_index, num_cols, u.head(num_rows), b);
    algebra::ApplyHouseholder(_H, row_index, start_col_index_dense_part, _cols_dense_part, u.head(num_rows), b);
    algebra::ApplyHouseholder1(_z, row_index, u.head(num_rows), b);
//    std::cout << "(" << row_index << "," << col_index << ","
//              << num_rows << "," << num_cols << ")" << std::endl;
  }
  // Reduce the columns of the dense part.
  // Get the column that terminates the for-loop for the reduction of this ladder step.
  end_col = std::min((int)_H.cols(), _index_pair[_index_pair.size() - 1].first + zero_cols_front);
  for (int j = start_col_index_dense_part; j < end_col; j++) {
    int row_index = j - zero_cols_front;
    int col_index = j;
    int num_rows = _H.rows() - row_index;
    int num_cols = _H.cols() - j;
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(_H.block(row_index, col_index, num_rows, 1), u.head(num_rows), b);
    algebra::ApplyHouseholder(_H, row_index, col_index, num_cols, u.head(num_rows), b);
    algebra::ApplyHouseholder1(_z, row_index, u.head(num_rows), b);
//    std::cout << "(" << row_index << "," << col_index << ","
//              << num_rows << "," << num_cols << ")" << std::endl;
  }
}

/**
 * @brief The Measurement compression interface class.
 * The class does the booking for, and calls the MeasurementCompressionTransformation
 * function. It registers structure while the Jacobian constructed and compresses
 * the big Jacobian at the end.
 * Note: For every small jacobian considered for Measurement compression its structure
 * need to be registered with the same order that it was stacked to the big Jacobian.
 */
struct MeasurementCompressionInterface {
  int row_counter_;
  int num_col_groups_;
  int col_group_size_;
  std::vector<std::vector<int>> perm_guide_;
  std::vector<std::pair<int, int>> lad_indices_;
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixRXf;
  /**
   * @brief Initialize the class before registering any structure.
   */
  void Init(int _num_col_groups, int _col_group_size, int _rows_per_group_guess = -1) {
    row_counter_ = 0;
    num_col_groups_ = _num_col_groups;
    col_group_size_ = _col_group_size;
    perm_guide_.resize(num_col_groups_);
    for (unsigned int idx = 0; idx < perm_guide_.size(); idx++) {
      perm_guide_[idx].clear();
    }
    if (_rows_per_group_guess != -1) {
      for (int i = 0; i < _num_col_groups - 1; i++) {
        perm_guide_[i].reserve(_rows_per_group_guess);
      }
    }
    lad_indices_.clear();
  }
  /**
   * @brief Registers the structure of each small Jacobian, considered for compression.
   */
  void RegisterStructure(int _rows, int _rows_first_lad_step,
                         int _rows_lad_step, int _init_col_group) {
    // Register the structure of the first block row. Makes more sense when the
    // _rows_first_lad_step is different than _rows_lad_step.
    for (int idx_row = 0; idx_row < _rows_first_lad_step; idx_row++) {
      perm_guide_.at(_init_col_group).push_back(row_counter_++);
    }
    // Register the structure of the remaining block rows.
    int rows_remain = _rows - _rows_first_lad_step;
    for (int idx_row = 0; idx_row < rows_remain; idx_row++) {
      perm_guide_.at(_init_col_group + 1 + idx_row / _rows_lad_step).push_back(row_counter_++);
    }
  }
  /**
   * @brief Registers the structure of each small Jacobian, considered for compression.
   */
  void RegisterStructure(const std::vector<std::pair<int, int>> &_corners) {
    for (int row = 0, corner_index = 0; corner_index < _corners.size(); ++row) {
      perm_guide_.at(_corners[corner_index].second / col_group_size_).push_back(row_counter_);
      ++row_counter_;
      if (row == _corners[corner_index].first) {
        ++corner_index;
      }
    }
  }
  /**
  * @brief Performs a row permutation to reveil the structure of the big Jacobian,
  * and then it applies measurement compression, while zeroing out the part of
  * the big Jacobian that is numerical zeros (strictly lower triangular part).
  */
  void GenerateLadderPermute(Eigen::Ref<MatrixRXf> _H,
                            Eigen::Ref<Eigen::VectorXf> _z,
                            Eigen::Ref<MatrixRXf> _H_compr,
                            Eigen::Ref<Eigen::VectorXf> _z_compr) {
    // Row permutate the big jacobian to reveil structure.
    int row_idx_copy = 0;
    for (unsigned int idx_perm_grp = 0; idx_perm_grp < perm_guide_.size(); idx_perm_grp++) {
      if (perm_guide_[idx_perm_grp].size() > 0) {
        for (unsigned int idx_in_grp = 0; idx_in_grp < perm_guide_[idx_perm_grp].size(); idx_in_grp++) {
          _H_compr.row(row_idx_copy) = _H.row(perm_guide_[idx_perm_grp][idx_in_grp]);
          _z_compr(row_idx_copy) = _z(perm_guide_[idx_perm_grp][idx_in_grp]);
          row_idx_copy++;
        }
        lad_indices_.emplace_back(row_idx_copy - 1, idx_perm_grp * col_group_size_);
      }
    }
  }

  void ApplyCompressionTransformation(Eigen::Ref<MatrixRXf> _H_compr,
                                      Eigen::Ref<Eigen::VectorXf> _z_compr,
                                      int _zero_cols_back = 0,
                                      int _dense_cols_end = 0) {
    // Measurement compression function call.
    algebra::MeasurmentCompressionTransformation(
        _H_compr, _z_compr, lad_indices_, _dense_cols_end, _zero_cols_back);
    int zeros_cols_front = lad_indices_[0].second;
    _H_compr.topRightCorner(std::min(_H_compr.rows(), _H_compr.cols() - zeros_cols_front),
                            _H_compr.cols() - zeros_cols_front).
        triangularView<Eigen::StrictlyLower>().setZero();
  }
};


/// @fn LowRankFactorUpdate, Perform low-rank update of an upper triangular factor _R_prev
/// with a dense matrix _V, by performing inplace Givens QR on the matrix R_
/// formed after stacking R_prev and _V appropriately and update z_ accordingly.
/// Note: The function applies for matrices stored row-wise.
inline void LowRankFactorUpdate(Eigen::Ref<Eigen::Matrix<float,
                    Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _R_prev,
                    const Eigen::Ref< const Eigen::Matrix<float,
                    Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> & _V,
                    Eigen::Ref<Eigen::VectorXf> _z_R,
                    const Eigen::Ref< const Eigen::VectorXf> & _z_V,
                    int _zero_cols_behind_V = 0) {
  /// Construct the R matrix after stacking R_prev and V and then apply a row Permutation.
  Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> R =
    Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(
        _R_prev.rows() + _V.rows(), _R_prev.cols());
  Eigen::VectorXf z = Eigen::VectorXf::Zero(_R_prev.rows() + _V.rows());

  /// Size of the upper portion of R that will be stacked on top.
  int size_R_upper_portion = R.cols() - (_zero_cols_behind_V + _V.cols());
//  std::cout << size_R_upper_portion << std::endl;
  /// Stack the upper part of R.
  R.topLeftCorner(size_R_upper_portion, _R_prev.cols()).triangularView<Eigen::Upper>() =
      _R_prev.topLeftCorner(size_R_upper_portion, _R_prev.cols()).triangularView<Eigen::Upper>();
  z.head(size_R_upper_portion) = _z_R.head(size_R_upper_portion);

  /// Stack V.
  R.block(size_R_upper_portion, size_R_upper_portion, _V.rows(), _V.cols()) = _V;
  z.segment(size_R_upper_portion, _V.rows()) = _z_V;
  /// Stack the rest of R.
  R.block(size_R_upper_portion + _V.rows(), size_R_upper_portion,
          R.rows() - size_R_upper_portion - _V.rows(),
          _R_prev.cols() - size_R_upper_portion).triangularView<Eigen::Upper>() =
  _R_prev.bottomRightCorner(R.rows() - size_R_upper_portion - _V.rows(),
                            _R_prev.cols() - size_R_upper_portion).triangularView<Eigen::Upper>();
  z.segment(size_R_upper_portion + _V.rows(), R.rows() - size_R_upper_portion - _V.rows()) =
  _z_R.tail(R.rows() - size_R_upper_portion - _V.rows());
//    std::cout << "R_V_stack\n" << R << std::endl << std::endl;

  /// Constant-number-of-givens-rotations phase.
  /// For each column of the constant-number-of-givens-rotations phase.
  for (int j = size_R_upper_portion; j < R.rows() - _V.rows(); j++) {
    /// For each row of the constant-number-of-givens-rotations phase.
    for (int i = j + _V.rows() - 1; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(R(i, j), R(i + 1, j), c, s);
      algebra::ApplyGivens(R, i, j, R.cols() - j, c, s);
      algebra::ApplyGivens1(z, i, c, s);
//        std::cout << "(" << j << "," << i << "," << R.cols() - j << ")" << std::endl;
    }
  }

  /// Shinking--number-of-givens-rotations phase.
  /// For each column of the shrinking-number-of-givens-rotations phase.
  for (int j = R.rows() - _V.rows(); j < R.cols(); j++) {
    /// For each row of the shrinking-number-of-givens-rotations phase.
    for (int i = R.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(R(i, j), R(i + 1, j), c, s);
      algebra::ApplyGivens(R, i, j, R.cols() - j, c, s);
      algebra::ApplyGivens1(z, i, c, s);
//        std::cout << "(" << j << "," << i << "," << R.cols() - j << ")" << std::endl;
    }
  }

  _R_prev = R.topLeftCorner(_R_prev.rows(), _R_prev.cols());
  _z_R = z.head(_R_prev.rows());
}

/// @fn LowRankFactorTriuUpdate, Perform low-rank update of the upper triangular factor _R_prev
/// with an upper triangular matrix _V, by performing inplace Givens QR on the matrix R_
/// formed after stacking R_prev and _V appropriately, and update z_ accordingly.
/// Note: The function applies for matrices stored row-wise.
inline void LowRankFactorTriuUpdate(Eigen::Ref<Eigen::Matrix<float,
                         Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _R_prev,
                         const Eigen::Ref< const Eigen::Matrix<float,
                         Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> & _V,
                         Eigen::Ref<Eigen::VectorXf> _z_R,
                         const Eigen::Ref< const Eigen::VectorXf> & _z_V,
                         int _zero_cols_behind_V = 0) {
  /// Construct the R matrix after stacking R_prev and V and then apply A row Permutation.
  Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> R =
    Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(
        _R_prev.rows() + _V.rows(), _R_prev.cols());
  Eigen::VectorXf z = Eigen::VectorXf::Zero(_R_prev.rows() + _V.rows());

  /// Size of the upper portion of R that will be stacked on top.
  int size_R_upper_portion = R.cols() - (_zero_cols_behind_V + _V.cols());
  /// Stack the upper part of R.
  R.topLeftCorner(size_R_upper_portion, _R_prev.cols()).triangularView<Eigen::Upper>() =
      _R_prev.topLeftCorner(size_R_upper_portion, _R_prev.cols()).triangularView<Eigen::Upper>();
  z.head(size_R_upper_portion) = _z_R.head(size_R_upper_portion);
  //  std::cout << size_R_upper_portion << std::endl;

  /// Alternate lines between V and R_prev.
  /// Get the minimum between the number of rows of V and the leftover of R_prev.
  bool V_min_rows = (_V.rows() <= _R_prev.rows() - size_R_upper_portion)? true : false;
  int min_rows = (V_min_rows)? _V.rows() : _R_prev.rows() - size_R_upper_portion;
  /// For each row in the ladder part.
  for (int i = 0; i < min_rows; i++) {
    /// Copy first the ith row of V.
    R.block(size_R_upper_portion + 2*i, size_R_upper_portion + i,
            1, _V.cols() - i) = _V.block(i, i, 1, _V.cols() - i);
    z(size_R_upper_portion + 2*i) = _z_V(i);
    /// Copy the the ith row of R_prev.
    R.block(size_R_upper_portion + 2*i + 1, size_R_upper_portion + i,
            1, _R_prev.cols() - (size_R_upper_portion + i)) =
    _R_prev.block(size_R_upper_portion + i, size_R_upper_portion + i,
                  1, _R_prev.cols() - (size_R_upper_portion + i));
    z(size_R_upper_portion + 2*i + 1) = _z_R(size_R_upper_portion + i);
  }

  /// Copy the leftover of either V or R_prev.
  int leftover_rows = R.rows() - (size_R_upper_portion + 2 * min_rows);
  int leftover_cols;
  if (V_min_rows) {
    leftover_cols = _R_prev.cols() - (size_R_upper_portion + min_rows);
    R.block(size_R_upper_portion + 2 * min_rows, size_R_upper_portion + min_rows,
            leftover_rows, leftover_cols).triangularView<Eigen::Upper>() =
        _R_prev.bottomRightCorner(leftover_rows, leftover_cols).triangularView<Eigen::Upper>();
    z.segment(size_R_upper_portion + 2 * min_rows,leftover_rows) = _z_R.tail(leftover_rows);
  }
  else {
    leftover_cols = _V.cols() - min_rows;
    R.block(size_R_upper_portion + 2 * min_rows, size_R_upper_portion + min_rows,
            leftover_rows, leftover_cols).triangularView<Eigen::Upper>() =
                _V.bottomRightCorner(leftover_rows, leftover_cols).triangularView<Eigen::Upper>();
    z.segment(size_R_upper_portion + 2 * min_rows,leftover_rows) = _z_V.tail(leftover_rows);
  }
//    std::cout << "R_V_stack\n" << R << std::endl << std::endl;

  /// Growing-number-of-givens-rotations phase.
  /// For each column of the growing-number-of-givens-rotations phase.
  for (int j = 0; j < min_rows; j++) {
    /// For each row of the growing-number-of-givens-rotations phase.
    for (int i = 2 * j; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(R(i + size_R_upper_portion, j + size_R_upper_portion),
                             R(i + size_R_upper_portion + 1, j + size_R_upper_portion), c, s);
      algebra::ApplyGivens(R, i + size_R_upper_portion, j + size_R_upper_portion,
                           R.cols() - (j + size_R_upper_portion), c, s);
      algebra::ApplyGivens1(z, i + size_R_upper_portion, c, s);
//      std::cout << "(" << j + size_R_upper_portion << "," << i + size_R_upper_portion << ","
//                << R.cols() - (j + size_R_upper_portion) << ")" << std::endl;
    }
  }

  /// Constant-number-of-givens-rotations phase.
  /// For each column of the constant-number-of-givens-rotations phase.
  for (int j = min_rows; j < min_rows + leftover_rows; j++) {
    /// For each row of the constant-number-of-givens-rotations phase.
    for (int i = 2 * j - (j - min_rows) - 1; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(R(i + size_R_upper_portion, j + size_R_upper_portion),
                             R(i + size_R_upper_portion + 1, j + size_R_upper_portion), c, s);
      algebra::ApplyGivens(R, i + size_R_upper_portion, j + size_R_upper_portion,
                           R.cols() - (j + size_R_upper_portion), c, s);
      algebra::ApplyGivens1(z, i + size_R_upper_portion, c, s);
//        std::cout << "(" << j + size_R_upper_portion << "," << i + size_R_upper_portion << ","
//                  << R.cols() - (j + size_R_upper_portion) << ")" << std::endl;
    }
  }

  /// Shinking--number-of-givens-rotations phase.
  /// For each column of the shrinking-number-of-givens-rotations phase.
  for (int j = size_R_upper_portion + min_rows + leftover_rows;
      j < size_R_upper_portion + min_rows + leftover_cols; j++) {
    /// For each row of the shrinking-number-of-givens-rotations phase.
    for (int i = R.rows() - 2; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(R(i, j), R(i + 1, j), c, s);
      algebra::ApplyGivens(R, i, j, R.cols() - j, c, s);
      algebra::ApplyGivens1(z, i, c, s);
//        std::cout << "(" << j << "," << i << "," << R.cols() - j << ")" << std::endl;
    }
  }

    _R_prev = R.topLeftCorner(_R_prev.rows(), _R_prev.cols());
    _z_R = z.head(_R_prev.rows());
}

inline void LowRankFactorTriuUpdate_Full(Eigen::Ref<Eigen::Matrix<float,
                         Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _R_prev,
                         Eigen::Ref<Eigen::Matrix<float,
                         Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _V,
						 Eigen::Ref<Eigen::Matrix<float,
						 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _PR_prev,
						 Eigen::Ref<Eigen::Matrix<float,
						 Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _PV,
                         Eigen::Ref<Eigen::VectorXf> _z_R,
                         Eigen::Ref<Eigen::VectorXf> _z_V,
						 int _zero_cols_front = 0) {
  /// Construct the R matrix after stacking R_prev and V and then apply A row Permutation.
  Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> R =
    Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(
        _R_prev.rows() + _V.rows(), _R_prev.cols());
  Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> PR =
      Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(
          _PR_prev.rows() + _PV.rows(), _PR_prev.cols());
  Eigen::VectorXf z = Eigen::VectorXf::Zero(_R_prev.rows() + _V.rows());

  /// Alternate lines between V and R_prev.
  int min_rows = _V.rows();
  /// For each row in the ladder part.
  for (int i = 0; i < min_rows; i++) {
    /// Copy first the ith row of V.
    R.block(2*i, i, 1, _V.cols() - i) = _V.block(i, i, 1, _V.cols() - i);
    PR.block(2*i, _zero_cols_front, 1, _PV.cols() - _zero_cols_front) = _PV.block(i, _zero_cols_front, 1, _PV.cols() - _zero_cols_front);
    z(2*i) = _z_V(i);
    /// Copy the the ith row of R_prev.
    R.block(2*i + 1, i, 1, _R_prev.cols() - i) = _R_prev.block(i, i, 1, _R_prev.cols() - i);
    PR.block(2*i + 1, _zero_cols_front, 1, _PR_prev.cols() - _zero_cols_front) = _PR_prev.block(i, _zero_cols_front, 1, _PR_prev.cols() - _zero_cols_front);
    z(2*i + 1) = _z_R(i);
  }

  /// Growing-number-of-givens-rotations phase.
  /// For each column of the growing-number-of-givens-rotations phase.
  for (int j = 0; j < min_rows; j++) {
    /// For each row of the growing-number-of-givens-rotations phase.
    for (int i = 2 * j; i >= j; i--) {
      float c, s;
      /// Compute the givens rotation operator and apply it accordingly.
      algebra::ComputeGivens(R(i, j), R(i + 1, j), c, s);
      algebra::ApplyGivens(R, i, j, R.cols() - j, c, s);
      algebra::ApplyGivens(PR, i, _zero_cols_front, PR.cols() - _zero_cols_front, c, s);

      algebra::ApplyGivens1(z, i, c, s);
//      std::cout << "(" << j + size_R_upper_portion << "," << i + size_R_upper_portion << ","
//                << R.cols() - (j + size_R_upper_portion) << ")" << std::endl;
    }
  }

    _R_prev = R.topLeftCorner(_R_prev.rows(), _R_prev.cols());
    _PR_prev = PR.topLeftCorner(_PR_prev.rows(), _PR_prev.cols());
    _z_R = z.head(_R_prev.rows());
    //_V is not copied back, since it is zero and is not used anywhere later
    _PV = PR.block(_PR_prev.rows(), 0, _PV.rows(), _PV.cols());
    _z_V = z.tail(_V.rows());
}

/// @fn LowRankFactorUpdateHouseholder, Perform low-rank update of an upper triangular factor _R_prev
/// with a dense matrix _V, by performing inplace Householder QR on the matrix R,
/// formed after stacking R_prev and _V appropriately, and update z_ accordingly.
/// Note: The function applies for matrices stored row-wise.
inline void LowRankFactorUpdateHouseholder(Eigen::Ref<Eigen::Matrix<float,
                    Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _R_prev,
                    const Eigen::Ref< const Eigen::Matrix<float,
                    Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> & _V,
                    Eigen::Ref<Eigen::VectorXf> _z_R,
                    const Eigen::Ref< const Eigen::VectorXf> & _z_V,
                    int _zero_cols_behind_V = 0) {
  /// Construct the R matrix after stacking R_prev and V and then apply A row Permutation.
  Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> R =
    Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(
        _R_prev.rows() + _V.rows(), _R_prev.cols());
  Eigen::VectorXf z = Eigen::VectorXf::Zero(_R_prev.rows() + _V.rows());

  /// Size of the upper portion of R that will be stacked on top.
  int size_R_upper_portion = R.cols() - (_zero_cols_behind_V + _V.cols());
//  std::cout << size_R_upper_portion << std::endl;
  /// Stack the upper part of R.
  R.topLeftCorner(size_R_upper_portion, _R_prev.cols()).triangularView<Eigen::Upper>() =
      _R_prev.topLeftCorner(size_R_upper_portion, _R_prev.cols()).triangularView<Eigen::Upper>();
  z.head(size_R_upper_portion) = _z_R.head(size_R_upper_portion);

  /// Stack V.
  R.block(size_R_upper_portion, size_R_upper_portion, _V.rows(), _V.cols()) = _V;
  z.segment(size_R_upper_portion, _V.rows()) = _z_V;
  /// Stack the rest of R.
  R.block(size_R_upper_portion + _V.rows(), size_R_upper_portion,
          R.rows() - size_R_upper_portion - _V.rows(),
          _R_prev.cols() - size_R_upper_portion).triangularView<Eigen::Upper>() =
  _R_prev.bottomRightCorner(R.rows() - size_R_upper_portion - _V.rows(),
                            _R_prev.cols() - size_R_upper_portion).triangularView<Eigen::Upper>();
  z.segment(size_R_upper_portion + _V.rows(), R.rows() - size_R_upper_portion - _V.rows()) =
      _z_R.tail(R.rows() - size_R_upper_portion - _V.rows());
//    std::cout << "R_V_stack\n" << R << std::endl << std::endl;

  // Householder reflection utilities.
  Eigen::VectorXf u(_V.rows() + 1);
  float b;

  /// Constant-number-of-householder-reflections phase.
  /// For each column of the constant-number-of-householder-reflections phase.
  for (int j = size_R_upper_portion; j < R.rows() - _V.rows(); j++) {
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(R.block(j, j, _V.rows() + 1, 1), u.head(_V.rows() + 1), b);
    algebra::ApplyHouseholder(R, j, j, R.cols() - j, u.head(_V.rows() + 1), b);
    algebra::ApplyHouseholder1(z, j, u.head(_V.rows() + 1), b);
//      std::cout << "(" << j << "," << j << "," << _V.rows() + 1 << ","
//                << R.cols() - (j + min_zero_cols_behind) << ")" << std::endl;
  }

  /// Shinking--number-of-householder-reflections phase.
  /// For each column of the shrinking-number-of-householder-reflections phase.
  for (int j = R.rows() - _V.rows(); j < R.cols(); j++) {
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(R.block(j, j, R.rows() - j, 1), u.head(R.rows() - j), b);
    algebra::ApplyHouseholder(R, j, j, R.cols() - j, u.head(R.rows() - j), b);
    algebra::ApplyHouseholder1(z, j, u.head(R.rows() - j), b);
//    std::cout << "(" << j << "," << j << "," << R.rows() - j << ","
//              << R.cols() - (j + min_zero_cols_behind) << ")" << std::endl;
  }

//  R.conservativeResize(_R_prev.cols() + _zero_cols_behind_R, Eigen::NoChange);
//  return R;
  _R_prev = R.topLeftCorner(_R_prev.rows(), _R_prev.cols());
  _z_R = z.head(_R_prev.rows());
}

/// @fn LowRankFactorTriuUpdateHouseholder, Perform low-rank update of the upper triangular factor _R_prev
/// with an upper triangular matrix _V, by performing inplace Householder QR on the matrix R_
/// formed after stacking R_prev and _V appropriately, and update z_ accordingly.
/// Note: The function applies for matrices stored row-wise.
inline void LowRankFactorTriuUpdateHouseholder(Eigen::Ref<Eigen::Matrix<float,
                         Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> _R_prev,
                         const Eigen::Ref< const Eigen::Matrix<float,
                         Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> & _V,
                         Eigen::Ref<Eigen::VectorXf> _z_R,
                         const Eigen::Ref< const Eigen::VectorXf> & _z_V,
                         int _zero_cols_behind_V = 0) {
  /// Construct the R matrix after stacking R_prev and V and then apply A row Permutation.
  Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> R =
    Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(
        _R_prev.rows() + _V.rows(), _R_prev.cols());
  Eigen::VectorXf z = Eigen::VectorXf::Zero(_R_prev.rows() + _V.rows());

  /// Size of the upper portion of R that will be stacked on top.
  int size_R_upper_portion = R.cols() - (_zero_cols_behind_V + _V.cols());
  /// Stack the upper part of R.
  R.topLeftCorner(size_R_upper_portion, _R_prev.cols()).triangularView<Eigen::Upper>() =
      _R_prev.topLeftCorner(size_R_upper_portion, _R_prev.cols()).triangularView<Eigen::Upper>();
  z.head(size_R_upper_portion) = _z_R.head(size_R_upper_portion);
  //  std::cout << size_R_upper_portion << std::endl;

  /// Alternate lines between V and R_prev.
  /// Get the minimum between the number of rows of V and the leftover of R_prev.
  bool V_min_rows = (_V.rows() <= _R_prev.rows() - size_R_upper_portion)? true : false;
  int min_rows = (V_min_rows)? _V.rows() : _R_prev.rows() - size_R_upper_portion;
  /// For each row in the ladder part.
  for (int i = 0; i < min_rows; i++) {
    /// Copy first the ith row of V.
    R.block(size_R_upper_portion + 2*i, size_R_upper_portion + i,
            1, _V.cols() - i) = _V.block(i, i, 1, _V.cols() - i);
    z(size_R_upper_portion + 2*i) = _z_V(i);
    /// Copy the the ith row of R_prev.
    R.block(size_R_upper_portion + 2*i + 1, size_R_upper_portion + i,
            1, _R_prev.cols() - (size_R_upper_portion + i)) =
    _R_prev.block(size_R_upper_portion + i, size_R_upper_portion + i,
                  1, _R_prev.cols() - (size_R_upper_portion + i));
    z(size_R_upper_portion + 2*i + 1) =
    _z_R(size_R_upper_portion + i);
  }

  /// Copy the leftover of either V or R_prev.
  int leftover_rows = R.rows() - (size_R_upper_portion + 2 * min_rows);
  int leftover_cols;
  if (V_min_rows) {
    leftover_cols = _R_prev.cols() - (size_R_upper_portion + min_rows);
    R.block(size_R_upper_portion + 2 * min_rows, size_R_upper_portion + min_rows,
            leftover_rows, leftover_cols).triangularView<Eigen::Upper>() =
        _R_prev.bottomRightCorner(leftover_rows, leftover_cols).triangularView<Eigen::Upper>();
    z.segment(size_R_upper_portion + 2 * min_rows,leftover_rows) = _z_R.tail(leftover_rows);
  }
  else {
    leftover_cols = _V.cols() - min_rows;
    R.block(size_R_upper_portion + 2 * min_rows, size_R_upper_portion + min_rows,
            leftover_rows, leftover_cols).triangularView<Eigen::Upper>() =
                _V.bottomRightCorner(leftover_rows, leftover_cols).triangularView<Eigen::Upper>();
    z.segment(size_R_upper_portion + 2 * min_rows,leftover_rows) = _z_V.tail(leftover_rows);
  }
//    std::cout << "R_V_stack\n" << R << std::endl << std::endl;

  // Householder reflection utilities.
  Eigen::VectorXf u(min_rows + 1);
  float b;

  /// Growing-number-of-Householder-reflections phase.
  /// For each column of the growing-number-of-givens-rotations phase.
  for (int j = 0; j < min_rows; j++) {
      /// Compute the Householder reflection operator and apply it accordingly.
      algebra::ComputeHouseholder(R.block(size_R_upper_portion + j, size_R_upper_portion + j, j + 2, 1),
                                 u.head(j + 2), b);
      algebra::ApplyHouseholder(R, size_R_upper_portion + j, size_R_upper_portion + j,
                               R.cols() - (j + size_R_upper_portion), u.head(j + 2), b);
      algebra::ApplyHouseholder1(z, size_R_upper_portion + j, u.head(j + 2), b);
//      std::cout << "(" << j + size_R_upper_portion << "," << j + size_R_upper_portion << ","
//                << R.cols() - (j + size_R_upper_portion + min_zero_cols_behind) << ")" << std::endl;
  }

  /// Constant-number-of-givens-rotations phase.
  /// For each column of the constant-number-of-givens-rotations phase.
  for (int j = min_rows; j < min_rows + leftover_rows; j++) {
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(R.block(size_R_upper_portion + j, size_R_upper_portion + j, min_rows + 1, 1),
                               u.head(min_rows + 1), b);
    algebra::ApplyHouseholder(R, size_R_upper_portion + j, size_R_upper_portion + j,
                             R.cols() - (j + size_R_upper_portion), u.head(min_rows + 1), b);
    algebra::ApplyHouseholder1(z, size_R_upper_portion + j, u.head(min_rows + 1), b);
//      std::cout << "(" << j + size_R_upper_portion << "," << j + size_R_upper_portion << ","
//                << R.cols() - (j + size_R_upper_portion + min_zero_cols_behind) << ")" << std::endl;
  }

  /// Shinking--number-of-givens-rotations phase.
  /// For each column of the shrinking-number-of-givens-rotations phase.
  for (int j = size_R_upper_portion + min_rows + leftover_rows;
      j < size_R_upper_portion + min_rows + leftover_cols; j++) {
    /// Compute the Householder reflection operator and apply it accordingly.
    algebra::ComputeHouseholder(R.block(j, j, R.rows() - j, 1), u.head(R.rows() - j), b);
    algebra::ApplyHouseholder(R, j, j, R.cols() - j, u.head(R.rows() - j), b);
    algebra::ApplyHouseholder1(z, j, u.head(R.rows() - j), b);
//      std::cout << "(" << j << "," << j << ","
//                << R.cols() - (j + min_zero_cols_behind) << ")" << std::endl;
  }

//  R.conservativeResize(_R_prev.cols() + _zero_cols_behind_R, Eigen::NoChange);
//  return R;
  _R_prev = R.topLeftCorner(_R_prev.rows(), _R_prev.cols());
  _z_R = z.head(_R_prev.rows());
}

} /** End of namespace: algebra */
#endif
