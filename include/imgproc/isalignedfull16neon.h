#ifndef IS_ALIGNED_FULL_16_NEON_H_
#define IS_ALIGNED_FULL_16_NEON_H_

#ifndef F_PI
#define F_PI 3.1415927
#endif

#ifndef F_2__PI
#define F_2__PI 6.2831854
#endif

/**
* Function: IsAlignedFull16NEON
*
* 	Angle alignement check for 16 angles up to some accuracy
*
* @param angle  first operand (pointer to array of 16 float angles (float*))
* @param theta  second operand (float angle theta (float))
* @param accuracy  third operant (float scalar (float))
* @param result  result (pointer to array of 16 integer values 0 or -1 (int*))
*/
inline void IsAlignedFull16NEON(float * angles, float theta, float accuracy, int * result) {	
  float two_pi_minus_accuracy = F_2__PI - accuracy;
  float pi_minus_accuracy = F_PI - accuracy;
  float pi_plus_accuracy = F_PI + accuracy;
  asm (
    /// Load Constants
    "vdup.32 q2, %y[ac];"
    "vdup.32 q3, %y[twopi_m_ac];"
    "vdup.32 q4, %y[pi_m_ac];"
    "vdup.32 q5, %y[pi_p_ac];"
    "vdup.32 q6, %y[th];"
    /// Load 16 values of angles    
    "vld1.32 {d24, d25, d26, d27}, [%[angl]]!;"
    "vld1.32 {d28, d29, d30, d31}, [%[angl]];"
    ///---------------------- For angles[0:3]-----------------------------------  
    /// (theta - angles)
    "vsub.f32 q9, q6, q12;"
    /// |theta - angles| >= (pi - accuracy) ?  
    "vacge.f32 q10, q9, q4;"
    /// |theta - angles| <= (pi + accuracy) ?  
    "vacge.f32 q11, q5, q9;"
    ///(case 3): |theta - angles| >= (pi - accuracy) && |theta - angles| <= (pi + accuracy)
    "vand.i32 q11, q10, q11;"
    ///(case 2): |theta - angles| >= (2*pi - accuracy) ?
    "vacge.f32 q10, q9, q3;"
    /// (case 3) || (case 2)?
    "vorr.i32 q11, q10, q11;"
    /// (case 1): |theta - angles| <= accuracy
    "vacge.f32 q10, q2, q9;"
    /// (case 3) || (case 2) || (case 1) ?
    "vorr.i32 q11, q10, q11;"
    /// Store the result
    "vst1.32 {q11}, [%[res]]!;"
    ///---------------------- For angles[4:7]-----------------------------------  
    /// (theta - angles)
    "vsub.f32 q9, q6, q13;"
    /// |theta - angles| >= (pi - accuracy) ?  
    "vacge.f32 q10, q9, q4;"
    /// |theta - angles| <= (pi + accuracy) ?  
    "vacge.f32 q11, q5, q9;"
    ///(case 3): |theta - angles| >= (pi - accuracy) && |theta - angles| <= (pi + accuracy)
    "vand.i32 q11, q10, q11;"
    ///(case 2): |theta - angles| >= (2*pi - accuracy) ?
    "vacge.f32 q10, q9, q3;"
    /// (case 3) || (case 2)?
    "vorr.i32 q11, q10, q11;"
    /// (case 1): |theta - angles| <= accuracy
    "vacge.f32 q10, q2, q9;"
    /// (case 3) || (case 2) || (case 1) ?
    "vorr.i32 q11, q10, q11;"
    /// Store the result
    "vst1.32 {q11}, [%[res]]!;"
    ///---------------------- For angles[8:11]---------------------------------- 
    /// (theta - angles)
    "vsub.f32 q9, q6, q14;"
    /// |theta - angles| >= (pi - accuracy) ?  
    "vacge.f32 q10, q9, q4;"
    /// |theta - angles| <= (pi + accuracy) ?  
    "vacge.f32 q11, q5, q9;"
    ///(case 3): |theta - angles| >= (pi - accuracy) && |theta - angles| <= (pi + accuracy)
    "vand.i32 q11, q10, q11;"
    ///(case 2): |theta - angles| >= (2*pi - accuracy) ?
    "vacge.f32 q10, q9, q3;"
    /// (case 3) || (case 2)?
    "vorr.i32 q11, q10, q11;"
    /// (case 1): |theta - angles| <= accuracy
    "vacge.f32 q10, q2, q9;"
    /// (case 3) || (case 2) || (case 1) ?
    "vorr.i32 q11, q10, q11;"
    /// Store the result
    "vst1.32 {q11}, [%[res]]!;"
    ///---------------------- For angles[12:15]---------------------------------  
    /// (theta - angles)
    "vsub.f32 q9, q6, q15;"
    /// |theta - angles| >= (pi - accuracy) ?  
    "vacge.f32 q10, q9, q4;"
    /// |theta - angles| <= (pi + accuracy) ?  
    "vacge.f32 q11, q5, q9;"
    ///(case 3): |theta - angles| >= (pi - accuracy) && |theta - angles| <= (pi + accuracy)
    "vand.i32 q11, q10, q11;"
    ///(case 2): |theta - angles| >= (2*pi - accuracy) ?
    "vacge.f32 q10, q9, q3;"
    /// (case 3) || (case 2)?
    "vorr.i32 q11, q10, q11;"
    /// (case 1): |theta - angles| <= accuracy
    "vacge.f32 q10, q2, q9;"
    /// (case 3) || (case 2) || (case 1) ?
    "vorr.i32 q11, q10, q11;"
    /// Store the result
    "vst1.32 {q11}, [%[res]];"
    : /// no output
    : [angl] "r" (angles), [res] "r" (result), [ac] "w" (accuracy), 
      [th] "w" (theta), [twopi_m_ac] "w" (two_pi_minus_accuracy), 
      [pi_m_ac] "w" (pi_minus_accuracy), [pi_p_ac] "w" (pi_plus_accuracy)
    : "q2", "q3", "q4", "q5", "q6", "q9", "q10", "q11", "q12", "q13", "q14", "q15"
  );
}
#endif  /// IS_ALIGNED_FULL_16_NEON_H_
