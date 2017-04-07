#ifndef BLURRINESS_DETECTION_H_
#define BLURRINESS_DETECTION_H_ 

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include "core/matrix2d.h"

#include "imgproc/blurrinessPrecomp.h"

#include <math.h>

namespace EyeMARS {

class BlurrinessDetector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BlurrinessDetector() {
    for (int i = 0; i < EyeMars::BlurrinessPrecomp::num; i++) {
      sv_.emplace_back(EyeMars::BlurrinessPrecomp::support_vectors[2*i], EyeMars::BlurrinessPrecomp::support_vectors[2*i + 1]);
      
      w_.emplace_back(EyeMars::BlurrinessPrecomp::weights[i]);
    }

    shift << EyeMars::BlurrinessPrecomp::shiftx, EyeMars::BlurrinessPrecomp::shifty;
    scaleFactor << EyeMars::BlurrinessPrecomp::scalex, EyeMars::BlurrinessPrecomp::scaley;
    bias = EyeMars::BlurrinessPrecomp::bias;
    sigma = EyeMars::BlurrinessPrecomp::sigma;
  }

  BlurrinessDetector(std::string vector_file) : 
    vector_file(vector_file) {load(vector_file);}

  double decide(Matrix2d<short> & gradient);
  double decide(short* dst, int height, int width);
  double classify(const Eigen::Vector2f& query) {
    Eigen::Vector2f transformed = scaleFactor.cwiseProduct(query + shift);
    double c = 0;
    for (unsigned int i = 0; i < sv_.size(); i++) {
      double v = rbf(transformed, sv_[i], sigma);
      c += w_[i]*v;
    }
    c += bias;
    return (c);
  }

  void comp(BlurrinessDetector& d) {
    for (unsigned int i = 0; i < sv_.size(); i++) {
      float dist_vec = (sv_[i] - d.sv_[i]).norm();
      float dist_alpha = w_[i] - d.w_[i];
      if (dist_vec > 0 || dist_alpha > 0) {
        std::cout << "Difference[" << i << "] " <<std::endl;
        std::cout << sv_[i] << std::endl;
        std::cout << d.sv_[i] << std::endl;
        std::cout << w_[i] << " vs. " << d.w_[i] << std::endl;
        exit(-1);
      }
    }
  }

 private:
  std::string vector_file;
  std::vector<Eigen::Vector2f> sv_;
  std::vector<float> w_;
  Eigen::Vector2f shift;
  Eigen::Vector2f scaleFactor;
  float bias;

  void load(const std::string& path);

  double sigma;
  double rbf(const Eigen::Vector2f& v,
            const Eigen::Vector2f& u,
            double sigma) {
    double dist = v.squaredNorm() + u.squaredNorm() - 2*u.transpose()*v;
    return exp(-(1/(2*sigma*sigma))*dist);
  }
};

} // namespace EyeMars

#endif  // BLURRINESS_DETECTION_H_
