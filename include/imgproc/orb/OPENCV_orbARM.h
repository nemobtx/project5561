static unsigned char andPattern[16] =
{
  1,2,4,8,16,32,64,128,
  1,2,4,8,16,32,64,128
};



inline void computeARM(int dsize, const cv::Point* pattern, uchar* desc, float a, float b, int step, const uchar* center)
{
	uchar *andPattern0= (uchar*) andPattern;
	dsize=dsize>>1;
	asm volatile(
		"vdup.32 q7, %[center];"
		"vld1.8 {q8}, [%[andPattern]];"
	    
	      "Loop: ;"
		//loop 1
		
		"vld2.32 {d20,d21,d22,d23}, [%[r0]]!;"
		"vcvt.f32.s32 q10, q10;"
		"vcvt.f32.s32 q11, q11;"
	
		//Round(pattern[idx].x*b + pattern[idx].y*a)*step
		"vmul.f32 q2, q10, %y[b];"
		"vmla.f32 q2, q11, %y[a];"
		"vcvtr.s32.f32 s8, s8;"
		"vcvtr.s32.f32 s9, s9;"
		"vcvtr.s32.f32 s10, s10;"
		"vcvtr.s32.f32 s11, s11;"
		"vmul.i32 q2, q2, %y[step];"
		
		//Round((pattern[idx].x*a - pattern[idx].y*b))
		"vmul.f32 q1, q10, %y[a];"
		"vmls.f32 q1, q11, %y[b];"
		"vcvtr.s32.f32 s4, s4;"
		"vcvtr.s32.f32 s5, s5;"
		"vcvtr.s32.f32 s6, s6;"
		"vcvtr.s32.f32 s7, s7;"
		
		"vadd.i32 q1, q1, q2;"
		"vadd.i32 q1, q1, q7;"
		
		"vmov.32 r2, d2[0];"
		"vld1.8 {d6[0]}, [r2];"
		
		"vmov.32 r2, d2[1];"
		"vld1.8 {d8[0]}, [r2];"
		
		"vmov.32 r2, d3[0];"
		"vld1.8 {d6[1]}, [r2];"
		
		"vmov.32 r2, d3[1];"
		"vld1.8 {d8[1]}, [r2];"
		
		//loop 2
		"vld2.32 {d20,d21,d22,d23}, [%[r0]]!;"
		"vcvt.f32.s32 q10, q10;"
		"vcvt.f32.s32 q11, q11;"
		
		//Round(pattern[idx].x*b + pattern[idx].y*a)*step
		"vmul.f32 q2, q10, %y[b];"
		"vmla.f32 q2, q11, %y[a];"
		"vcvtr.s32.f32 s8, s8;"
		"vcvtr.s32.f32 s9, s9;"
		"vcvtr.s32.f32 s10, s10;"
		"vcvtr.s32.f32 s11, s11;"
		"vmul.i32 q2, q2, %y[step];"
		
		//Round((pattern[idx].x*a - pattern[idx].y*b))
		"vmul.f32 q1, q10, %y[a];"
		"vmls.f32 q1, q11, %y[b];"
		"vcvtr.s32.f32 s4, s4;"
		"vcvtr.s32.f32 s5, s5;"
		"vcvtr.s32.f32 s6, s6;"
		"vcvtr.s32.f32 s7, s7;"
		
		"vadd.i32 q1, q1, q2;"
		"vadd.i32 q1, q1, q7;"
		
		"vmov.32 r2, d2[0];"
		"vld1.8 {d6[2]}, [r2];"
		
		"vmov.32 r2, d2[1];"
		"vld1.8 {d8[2]}, [r2];"
		
		"vmov.32 r2, d3[0];"
		"vld1.8 {d6[3]}, [r2];"
		
		"vmov.32 r2, d3[1];"
		"vld1.8 {d8[3]}, [r2];"
		
		//loop 3
		"vld2.32 {d20,d21,d22,d23}, [%[r0]]!;"
		"vcvt.f32.s32 q10, q10;"
		"vcvt.f32.s32 q11, q11;"
		
		//Round(pattern[idx].x*b + pattern[idx].y*a)*step
		"vmul.f32 q2, q10, %y[b];"
		"vmla.f32 q2, q11, %y[a];"
		"vcvtr.s32.f32 s8, s8;"
		"vcvtr.s32.f32 s9, s9;"
		"vcvtr.s32.f32 s10, s10;"
		"vcvtr.s32.f32 s11, s11;"
		"vmul.i32 q2, q2, %y[step];"
		
		//Round((pattern[idx].x*a - pattern[idx].y*b))
		"vmul.f32 q1, q10, %y[a];"
		"vmls.f32 q1, q11, %y[b];"
		"vcvtr.s32.f32 s4, s4;"
		"vcvtr.s32.f32 s5, s5;"
		"vcvtr.s32.f32 s6, s6;"
		"vcvtr.s32.f32 s7, s7;"
		
		"vadd.i32 q1, q1, q2;"
		"vadd.i32 q1, q1, q7;"
		
		"vmov.32 r2, d2[0];"
		"vld1.8 {d6[4]}, [r2];"
		
		"vmov.32 r2, d2[1];"
		"vld1.8 {d8[4]}, [r2];"
		
		"vmov.32 r2, d3[0];"
		"vld1.8 {d6[5]}, [r2];"
		
		"vmov.32 r2, d3[1];"
		"vld1.8 {d8[5]}, [r2];"
		
		//loop 4
		"vld2.32 {d20,d21,d22,d23}, [%[r0]]!;"
		"vcvt.f32.s32 q10, q10;"
		"vcvt.f32.s32 q11, q11;"
		
		//Round(pattern[idx].x*b + pattern[idx].y*a)*step
		"vmul.f32 q2, q10, %y[b];"
		"vmla.f32 q2, q11, %y[a];"
		"vcvtr.s32.f32 s8, s8;"
		"vcvtr.s32.f32 s9, s9;"
		"vcvtr.s32.f32 s10, s10;"
		"vcvtr.s32.f32 s11, s11;"
		"vmul.i32 q2, q2, %y[step];"
		
		//Round((pattern[idx].x*a - pattern[idx].y*b))
		"vmul.f32 q1, q10, %y[a];"
		"vmls.f32 q1, q11, %y[b];"
		"vcvtr.s32.f32 s4, s4;"
		"vcvtr.s32.f32 s5, s5;"
		"vcvtr.s32.f32 s6, s6;"
		"vcvtr.s32.f32 s7, s7;"
		
		"vadd.i32 q1, q1, q2;"
		"vadd.i32 q1, q1, q7;"
		
		"vmov.32 r2, d2[0];"
		"vld1.8 {d6[6]}, [r2];"
		
		"vmov.32 r2, d2[1];"
		"vld1.8 {d8[6]}, [r2];"
		
		"vmov.32 r2, d3[0];"
		"vld1.8 {d6[7]}, [r2];"
		
		"vmov.32 r2, d3[1];"
		"vld1.8 {d8[7]}, [r2];"
		
		//loop 5
		"vld2.32 {d20,d21,d22,d23}, [%[r0]]!;"
		"vcvt.f32.s32 q10, q10;"
		"vcvt.f32.s32 q11, q11;"
		
		//Round(pattern[idx].x*b + pattern[idx].y*a)*step
		"vmul.f32 q2, q10, %y[b];"
		"vmla.f32 q2, q11, %y[a];"
		"vcvtr.s32.f32 s8, s8;"
		"vcvtr.s32.f32 s9, s9;"
		"vcvtr.s32.f32 s10, s10;"
		"vcvtr.s32.f32 s11, s11;"
		"vmul.i32 q2, q2, %y[step];"
		
		//Round((pattern[idx].x*a - pattern[idx].y*b))
		"vmul.f32 q1, q10, %y[a];"
		"vmls.f32 q1, q11, %y[b];"
		"vcvtr.s32.f32 s4, s4;"
		"vcvtr.s32.f32 s5, s5;"
		"vcvtr.s32.f32 s6, s6;"
		"vcvtr.s32.f32 s7, s7;"
		
		"vadd.i32 q1, q1, q2;"
		"vadd.i32 q1, q1, q7;"
		
		"vmov.32 r2, d2[0];"
		"vld1.8 {d7[0]}, [r2];"
		
		"vmov.32 r2, d2[1];"
		"vld1.8 {d9[0]}, [r2];"
		
		"vmov.32 r2, d3[0];"
		"vld1.8 {d7[1]}, [r2];"
		
		"vmov.32 r2, d3[1];"
		"vld1.8 {d9[1]}, [r2];"
		
		//loop 6
		"vld2.32 {d20,d21,d22,d23}, [%[r0]]!;"
		"vcvt.f32.s32 q10, q10;"
		"vcvt.f32.s32 q11, q11;"
		
		//Round(pattern[idx].x*b + pattern[idx].y*a)*step
		"vmul.f32 q2, q10, %y[b];"
		"vmla.f32 q2, q11, %y[a];"
		"vcvtr.s32.f32 s8, s8;"
		"vcvtr.s32.f32 s9, s9;"
		"vcvtr.s32.f32 s10, s10;"
		"vcvtr.s32.f32 s11, s11;"
		"vmul.i32 q2, q2, %y[step];"
		
		//Round((pattern[idx].x*a - pattern[idx].y*b))
		"vmul.f32 q1, q10, %y[a];"
		"vmls.f32 q1, q11, %y[b];"
		"vcvtr.s32.f32 s4, s4;"
		"vcvtr.s32.f32 s5, s5;"
		"vcvtr.s32.f32 s6, s6;"
		"vcvtr.s32.f32 s7, s7;"
		
		"vadd.i32 q1, q1, q2;"
		"vadd.i32 q1, q1, q7;"
		
		"vmov.32 r2, d2[0];"
		"vld1.8 {d7[2]}, [r2];"
		
		"vmov.32 r2, d2[1];"
		"vld1.8 {d9[2]}, [r2];"
		
		"vmov.32 r2, d3[0];"
		"vld1.8 {d7[3]}, [r2];"
		
		"vmov.32 r2, d3[1];"
		"vld1.8 {d9[3]}, [r2];"
		
		//loop 7
		"vld2.32 {d20,d21,d22,d23}, [%[r0]]!;"
		"vcvt.f32.s32 q10, q10;"
		"vcvt.f32.s32 q11, q11;"
		
		//Round(pattern[idx].x*b + pattern[idx].y*a)*step
		"vmul.f32 q2, q10, %y[b];"
		"vmla.f32 q2, q11, %y[a];"
		"vcvtr.s32.f32 s8, s8;"
		"vcvtr.s32.f32 s9, s9;"
		"vcvtr.s32.f32 s10, s10;"
		"vcvtr.s32.f32 s11, s11;"
		"vmul.i32 q2, q2, %y[step];"
		
		//Round((pattern[idx].x*a - pattern[idx].y*b))
		"vmul.f32 q1, q10, %y[a];"
		"vmls.f32 q1, q11, %y[b];"
		"vcvtr.s32.f32 s4, s4;"
		"vcvtr.s32.f32 s5, s5;"
		"vcvtr.s32.f32 s6, s6;"
		"vcvtr.s32.f32 s7, s7;"
		
		"vadd.i32 q1, q1, q2;"
		"vadd.i32 q1, q1, q7;"
		
		"vmov.32 r2, d2[0];"
		"vld1.8 {d7[4]}, [r2];"
		
		"vmov.32 r2, d2[1];"
		"vld1.8 {d9[4]}, [r2];"
		
		"vmov.32 r2, d3[0];"
		"vld1.8 {d7[5]}, [r2];"
		
		"vmov.32 r2, d3[1];"
		"vld1.8 {d9[5]}, [r2];"
		
		//loop 8
		"vld2.32 {d20,d21,d22,d23}, [%[r0]]!;"
		"vcvt.f32.s32 q10, q10;"
		"vcvt.f32.s32 q11, q11;"
		
		//Round(pattern[idx].x*b + pattern[idx].y*a)*step
		"vmul.f32 q2, q10, %y[b];"
		"vmla.f32 q2, q11, %y[a];"
		"vcvtr.s32.f32 s8, s8;"
		"vcvtr.s32.f32 s9, s9;"
		"vcvtr.s32.f32 s10, s10;"
		"vcvtr.s32.f32 s11, s11;"
		"vmul.i32 q2, q2, %y[step];"
		
		//Round((pattern[idx].x*a - pattern[idx].y*b))
		"vmul.f32 q1, q10, %y[a];"
		"vmls.f32 q1, q11, %y[b];"
		"vcvtr.s32.f32 s4, s4;"
		"vcvtr.s32.f32 s5, s5;"
		"vcvtr.s32.f32 s6, s6;"
		"vcvtr.s32.f32 s7, s7;"
		
		"vadd.i32 q1, q1, q2;"
		"vadd.i32 q1, q1, q7;"
		
		"vmov.32 r2, d2[0];"
		"vld1.8 {d7[6]}, [r2];"
		
		"vmov.32 r2, d2[1];"
		"vld1.8 {d9[6]}, [r2];"
		
		"vmov.32 r2, d3[0];"
		"vld1.8 {d7[7]}, [r2];"
		
		"vmov.32 r2, d3[1];"
		"vld1.8 {d9[7]}, [r2];"
		
		//-----------------------//
		
		// using 16 + 16 values to perform 8 + 8 comparisons
		"vcgt.u8 q10, q4, q3;"
		"vand.i8 q10, q10, q8;"
		
		"vshl.i64 q11, q10, #32;"
		"vorr.i8 q10, q10, q11;"
		"vshl.i64 q11, q10, #16;"
		"vorr.i8 q10, q10, q11;"
		"vshl.i64 q11, q10, #8;"
		"vorr.i8 q10, q10, q11;"
		"vst2.8 {d20[7],d21[7]}, [%[r1]]!;"
	
		"subs %[dsize], %[dsize], #1;"
		"bne  Loop;"
		
		: 
		: [r0] "r" (pattern), [dsize] "r" (dsize), [a] "w" (a), [b] "w" (b), [step] "w" (step), [r1] "r" (desc), [center] "r" (center), [andPattern] "r" (andPattern0)
		: "q1", "q2", "q3", "q4", "q7", "q8", "q10", "q11", "r2", "cc"
	);
	
	//for(int i=0;i<4;i++)
	//	std::cout<<int(descArray[i])<<std::endl;
 }
