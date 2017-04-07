#ifndef __EyeMARS_KEYPOINT_H__
#define __EyeMARS_KEYPOINT_H__

namespace EyeMARS {
  
struct Keypoint {
  short scale;
  short octave;
  float x;
  float y;
  float x_refined;
  float y_refined;
  float scale_refined;
  float edge_score;
  float laplacian_score;
};

} // namespace EyeMARS
#endif // __EyeMARS_KEYPOINT_H__