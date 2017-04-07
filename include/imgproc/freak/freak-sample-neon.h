#ifndef FREAK_SAMPLE_NEON
#define FREAK_SAMPLE_NEON
void SampleEvenRingNeon(unsigned short *samples, float relx, float rely, float radius, float *R, unsigned short *img, int stride, float *dx, float *dy)
{	
	int width = stride>>1;
	// float Qform6 = 64;
	float toShort = 65536;

	asm volatile(

		//=======================================Initialize================================================//
		"vld1.32 {q0}, [%[dx]]!;" 	//q0 <- dx
		"vld1.32 {q1}, [%[dy]]!;" 	//q1 <- dy
		"vmov.f32 s8, #3204448256;" //=-0.5 <-dx
		"vmov.f32 s9, #3212836864;" //=-1   <-dx
		"vmov.32 d5, d4;"			//q2 <- dx[4], dx[5], dx[4], dx[5]
		"vrev64.32 d22, d3;"
		"vmov.32 d23, d22;"			//q11<- dy[4], dy[5], dy[4], dy[5]


		"vld1.32 {q3}, [%[R]]!;"  	//q3 <- R
		"vdup.32 q4, %[relx];" 		//q4 <- relx
		"vdup.32 q5, %[rely];" 		//q5 <- rely
		"vdup.32 q6, %[radius];"	//q6 <- radius
		//=================================================================================================//


		//===============================Find point floating point x,y=====================================//
		//x_final (0, 1, 2, 3)
		"vmul.f32 q7, q0, d6[0];"	//q7 <- dx*R[0]
		"vmul.f32 q7, q7, q6;"		//q7 <- (dx*R[0]) * radius
		"vadd.f32 q7, q7, q4;"		//q7 <- relx + (dx*R[0]) * radius
		"vmul.f32 q8, q1, d6[1];"	//q8 <- dy*R[1]
		"vmla.f32 q7, q8, q6;"		//q7 <- (relx + (dx*R[0]) * radius) + (dy*R[1]) * radius = x_final

		//y_final (0, 1, 2, 3)
		"vmul.f32 q8, q0, d7[0];"	//q8 <- dx*R[2]
		"vmul.f32 q8, q8, q6;"		//q8 <- (dx*R[2]) * radius
		"vadd.f32 q8, q8, q5;"		//q8 <- rely + (dx*R[2]) * radius
		"vmul.f32 q9, q1, d7[1];"	//q9 <- dy*R[3]
		"vmla.f32 q8, q9, q6;"		//q8 <- (rely + (dx*R[2]) * radius) + (dy*R[3]) * radius = y_final

		//x_final (4, 5) - y_final (4, 5)
		"vmov.32 q15, q3;"			//q15 <- R
		"vzip.32 q15, q3;"			
		"vswp d31, d6;"				//q15 <- [R[0], R[0], R[2], R[2]]	q3 <- [R[1], R[1], R[3], R[3]]
		"vmov.32 d9, d10;"			//q4 <- relx, relx, rely, rely

		"vmul.f32 q10, q15, q2;"	//q10<- (dx*R)
		"vmul.f32 q10, q10, q6;"	//q10<- (dx*R) * radius
		"vadd.f32 q9, q10, q4;"		//q9 <- rel[x/y] + (dx*R) * radius
		"vmul.f32 q10, q3, q11;"	//q10<- (dy*R)
		"vmla.f32 q9, q10, q6;"		//q9 <- (rel[x/y] + (dx*R) * radius) + (dy*R) * radius = [x_final, x_final, y_final, y_final]
		//=================================================================================================//

		//------Convert to ushort (Q format)------//
		// "vdup.32 q15, %[Qform6];"	//q15 <- 64
		// "vmul.f32 q7, q7, q15;"
		// "vmul.f32 q7, q7, q15;"
		//----------------------------------------//

		//===========================================Interpolate===========================================//
		"vdup.32 q11, %[img];"
		"vdup.32 q10, %[width];"

		//-----------------------------------------interpolate q-------------------------------------------//
		
		//--find distances (a1, a2)--//
		"vdup.32 q15, d1[0];"		//q15<- float(1)
		"vmov.u32 q14, #1;"			//q14<- int(1)
		"vcvt.u32.f32 q0, q7;"		//q0 <- floor(x_final) = int(xp)
		"vcvt.f32.u32 q1, q0;"		//q1 <- floor(x_final) = float(xp)
		"vcvt.u32.f32 q2, q8;"		//q2 <- floor(y_final) = int(yp)
		"vcvt.f32.u32 q3, q2;"	 	//q3 <- floor(y_final) = float(yp)

		"vsub.f32 q7, q7, q1;"		//q7 <- x_final - xp = a1
		"vsub.f32 q8, q8, q3;"		//q8 <- y_final - yp = a2

		"vsub.f32 q4, q15, q7;"		//q4 <- (1-a1)
		"vsub.f32 q5, q15, q8;"		//q5 <- (1-a2)
		//----------------------------//

		//-------read first row-------//
		"vmul.u32 q15, q10, q2;"	//q15<- int(yp) * width
		"vadd.u32 q15, q15, q0;"	//q15<- int(yp) * width + int(xp) = imgID1 / 2 (half ID for unsigned short)
		"vshl.u32 q15, q15, #1;"	//q15<- (int(yp) * width + int(xp)) * 2 = imgID1 

		"vadd.u32 q15, q11, q15;"	//q15<- img + imgID1
		"vmov.32 r2, d30[0];"
		"vld1.32 {d26[0]}, [r2];"	//q13<- img[imgID1]
		"vmov.32 r2, d30[1];"
		"vld1.32 {d26[1]}, [r2];"	//q13<- img[imgID1]
		"vmov.32 r2, d31[0];"
		"vld1.32 {d27[0]}, [r2];"	//q13<- img[imgID1]
		"vmov.32 r2, d31[1];"
		"vld1.32 {d27[1]}, [r2];"	//q13<- img[imgID1]

		"vuzp.16 d26, d27;"
		"vmovl.u16 q3, d26;"		//q3 <- img[imgID1]
		"vmovl.u16 q13, d27;"		//q13<- img[imgID1+1]
		//-----------------------------//

		"vdup.32 q1, %[toShort];"	//q1 <- 65536 (float to unsigned char)

		//--find w0--//
		"vmul.f32 q6, q4, q5;"		//q6 <- (1-a1)(1-a2) = w0
		"vmul.f32 q6, q6, q1;"
		"vcvt.u32.f32 q6, q6;"		//q6 <- w0 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmul.u32 q12, q6, q3;"		//q12<- w0*img[imgID1]
		//--------------------------------------//

		//--find w1--//
		"vmul.f32 q6, q7, q5;"		//q6 <- a1 * (1-a2) = w1
		"vmul.f32 q6, q6, q1;"
		"vcvt.u32.f32 q6, q6;"		//q6 <- w1 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 q12, q6, q13;"	//q12<- w0*img[imgID1] + w1*img[imgID1+1]
		//--------------------------------------//

		//-------read second row-------//
		"vadd.u32 q2, q2, q14;"		//q2 <- int(yp) + 1 -> (next row)
		"vmul.u32 q15, q10, q2;"	//q15<- (int(yp) + 1) * width
		"vadd.u32 q15, q15, q0;"	//q15<- (int(yp) + 1) * width + int(xp) = imgID2 / 2 (half ID for unsigned short)
		"vshl.u32 q15, q15, #1;"	//q15<- ((int(yp) + 1) * width + int(xp)) * 2 = imgID2 

		"vadd.u32 q15, q11, q15;"	//q15<- img + imgID2
		"vmov.32 r2, d30[0];"
		"vld1.32 {d26[0]}, [r2];"	//q13<- img[imgID2]
		"vmov.32 r2, d30[1];"
		"vld1.32 {d26[1]}, [r2];"	//q13<- img[imgID2]
		"vmov.32 r2, d31[0];"
		"vld1.32 {d27[0]}, [r2];"	//q13<- img[imgID2]
		"vmov.32 r2, d31[1];"
		"vld1.32 {d27[1]}, [r2];"	//q13<- img[imgID2]

		"vuzp.16 d26, d27;"
		"vmovl.u16 q3, d26;"		//q3 <- img[imgID2]
		"vmovl.u16 q13, d27;"		//q13<- img[imgID2+1]
		//------------------------------//

		//--find w2--//
		"vmul.f32 q6, q4, q8;"		//q6 <- (1-a1) * a2 = w2
		"vmul.f32 q6, q6, q1;"
		"vcvt.u32.f32 q6, q6;"		//q6 <- w2 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 q12, q6, q3;"	//q12<- w0*img[imgID1] + w1*img[imgID1+1] + w2*img[imgID2]
		//--------------------------------------//

		//--find w3--//
		"vmul.f32 q6, q7, q8;"		//q6 <- a1 * a2 = w3
		"vmul.f32 q6, q6, q1;"
		"vcvt.u32.f32 q6, q6;"		//q6 <- w3 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 q12, q6, q13;"	//q12<- w0*img[imgID1] + w1*img[imgID1+1] + w2*img[imgID2] + w3*img[imgID2+1]
		//--------------------------------------//

		"vrshrn.u32 d24, q12, #16;" //d24<-  unsigned short(interpolated value)
		"vst1.16 {d24}, [%[samples]]!;"
		//-------------------------------------------------------------------------------------------------//

		//-----------------------------------------interpolate d-------------------------------------------//

		//--find distances (a1, a2)--//
		"vmov.f32 s0, #3212836864;"	//s0=-1.0
		"vmul.f32 q0, q0, q0;"		//s0=1.0
		"vdup.32 d30, d0[0];"		//d30<- float(1)
		"vmov.u32 d28, #1;"			//d28<- int(1)//------------------------------------------(not needed!!! (test that))
		"vcvt.u32.f32 d0, d18;"		//d0 <- floor(x_final) = int(xp)
		"vcvt.f32.u32 d2, d0;"		//d2 <- floor(x_final) = float(xp)
		"vcvt.u32.f32 d4, d19;"		//d4 <- floor(y_final) = int(yp)
		"vcvt.f32.u32 d6, d4;"	 	//d6 <- floor(y_final) = float(yp)

		"vsub.f32 d14, d18, d2;"	//d14 <- x_final - xp = a1
		"vsub.f32 d16, d19, d6;"	//d16 <- y_final - yp = a2

		"vsub.f32 d8, d30, d14;"	//d8 <- (1-a1)
		"vsub.f32 d10, d30, d16;"	//d10<- (1-a2)
		//----------------------------//

		//-------read first row-------//
		"vmul.u32 d30, d20, d4;"	//q30<- int(yp) * width
		"vadd.u32 d30, d30, d0;"	//d30<- int(yp) * width + int(xp) = imgID1 / 2 (half ID for unsigned short)
		"vshl.u32 d30, d30, #1;"	//d30<- (int(yp) * width + int(xp)) * 2 = imgID1 

		"vadd.u32 d30, d22, d30;"	//d30<- img + imgID1
		"vmov.32 r2, d30[0];"
		"vld1.32 {d26[0]}, [r2];"	//q13<- img[imgID1]
		"vmov.32 r2, d30[1];"
		"vld1.32 {d26[1]}, [r2];"	//q13<- img[imgID1]

		"vrev32.16 d27, d26;"
		"vuzp.16 d26, d26;"
		"vuzp.16 d27, d27;"
		"vmovl.u16 q3, d26;"		//d6 <- img[imgID1]
		"vmovl.u16 q13, d27;"		//d26<- img[imgID1+1]
		//-----------------------------//   (good shape)

		"vdup.32 d2, %[toShort];"	//d2 <- 65536 (float to unsigned char)

		//--find w0--//
		"vmul.f32 d12, d8, d10;"	//d12<- (1-a1)(1-a2) = w0
		"vmul.f32 d12, d12, d2;"
		"vcvt.u32.f32 d12, d12;"	//d12<- w0 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmul.u32 d24, d12, d6;"	//d24<- w0*img[imgID1]
		//--------------------------------------//

		//--find w1--//
		"vmul.f32 d12, d14, d10;"	//d12<- a1 * (1-a2) = w1
		"vmul.f32 d12, d12, d2;"
		"vcvt.u32.f32 d12, d12;"	//d12 <- w1 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 d24, d12, d26;"	//d24<- w0*img[imgID1] + w1*img[imgID1+1]
		//--------------------------------------//

		//-------read second row-------//
		"vadd.u32 d4, d4, d28;"		//d4 <- int(yp) + 1 -> (next row)
		"vmul.u32 d30, d20, d4;"	//d30<- (int(yp) + 1) * width
		"vadd.u32 d30, d30, d0;"	//d30<- (int(yp) + 1) * width + int(xp) = imgID2 / 2 (half ID for unsigned short)
		"vshl.u32 d30, d30, #1;"	//d30<- ((int(yp) + 1) * width + int(xp)) * 2 = imgID2 

		"vadd.u32 d30, d22, d30;"	//d30<- img + imgID2
		"vmov.32 r2, d30[0];"
		"vld1.32 {d26[0]}, [r2];"	//q13<- img[imgID2]
		"vmov.32 r2, d30[1];"
		"vld1.32 {d26[1]}, [r2];"	//q13<- img[imgID2]

		"vrev32.16 d27, d26;"
		"vuzp.16 d26, d26;"
		"vuzp.16 d27, d27;"
		"vmovl.u16 q3, d26;"		//d6 <- img[imgID2]
		"vmovl.u16 q13, d27;"		//d26<- img[imgID2+1]
		//-----------------------------//

		//--find w2--//
		"vmul.f32 d12, d8, d16;"	//d12 <- (1-a1) * a2 = w2
		"vmul.f32 d12, d12, d2;"
		"vcvt.u32.f32 d12, d12;"	//d12 <- w2 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 d24, d12, d6;"	//d24<- w0*img[imgID1] + w1*img[imgID1+1] + w2*img[imgID2]
		//--------------------------------------//

		//--find w3--//
		"vmul.f32 d12, d14, d16;"	//d12 <- a1 * a2 = w3
		"vmul.f32 d12, d12, d2;"
		"vcvt.u32.f32 d12, d12;"	//d12 <- w3 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 d24, d12, d26;"	//d24<- w0*img[imgID1] + w1*img[imgID1+1] + w2*img[imgID2] + w3*img[imgID2+1]
		//--------------------------------------//

		"vrshr.u32 d24, d24, #16;" //d24<-  unsigned short(interpolated value)
		"vuzp.16 d24, d24;"
		"vst1.16 {d24}, [%[samples]]!;"
		//-------------------------------------------------------------------------------------------------//

		//=================================================================================================//

		

		: 
		: [samples] "r" (samples), [dx] "r" (dx), [dy] "r" (dy), [R] "r" (R), [relx] "r" (relx), [rely] "r" (rely),
		[radius] "r" (radius), [img] "r" (img), [width] "r" (width), [toShort] "r" (toShort)
		: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "q14", "q15", "r2"
	);


}

void SampleOddRingNeon(unsigned short *samples, float relx, float rely, float radius, float *R, unsigned short *img, int stride, float *dx, float *dy)
{	
	int width = stride>>1;
	// float Qform6 = 64;
	float toShort = 65536;
	
	asm volatile(

		//=========================================Initialize==============================================//
		"vld1.32 {q0}, [%[dx]];" 	//q0 <- dx[0], dx[1], dx[4], dx[5]
		"vdup.32 q2, d1[1];"		//q2 <- dx[4], dx[5], dx[4], dx[5] (dx[5]==dx[4])
		"vrev64.32 d1, d0;"			//q0 <- dx[0], dx[1], dx[2], dx[3] 

		"vld1.32 {q1}, [%[dy]];" 	//q1 <- dy
		"vmov.f32 s24, #3204448256;"//=-0.5 <-dy
		"vmov.f32 s25, #1056964608;"//= 0.5 <-dy
		"vmov.32 d13, d12;"			//q6 <- dy[4], dy[5], dy[4], dy[5]

		"vld1.32 {q3}, [%[R]]!;"  	//q3 <- R
		"vdup.32 q4, %[relx];" 		//q4 <- relx
		"vdup.32 q5, %[rely];" 		//q5 <- rely
		"vdup.32 q11, %[radius];"	//q11<- radius
		//=================================================================================================//


		// ===============================Find point floating point x,y=====================================//
		// x_final (0, 1, 2, 3)
		"vmul.f32 q7, q0, d6[0];"	//q7 <- dx*R[0]
		"vmul.f32 q7, q7, q11;"		//q7 <- (dx*R[0]) * radius
		"vadd.f32 q7, q7, q4;"		//q7 <- relx + (dx*R[0]) * radius
		"vmul.f32 q8, q1, d6[1];"	//q8 <- dy*R[1]
		"vmla.f32 q7, q8, q11;"		//q7 <- (relx + (dx*R[0]) * radius) + (dy*R[1]) * radius = x_final

		//y_final (0, 1, 2, 3)
		"vmul.f32 q8, q0, d7[0];"	//q8 <- dx*R[2]
		"vmul.f32 q8, q8, q11;"		//q8 <- (dx*R[2]) * radius
		"vadd.f32 q8, q8, q5;"		//q8 <- rely + (dx*R[2]) * radius
		"vmul.f32 q9, q1, d7[1];"	//q9 <- dy*R[3]
		"vmla.f32 q8, q9, q11;"		//q8 <- (rely + (dx*R[2]) * radius) + (dy*R[3]) * radius = y_final

		//x_final (4, 5) - y_final (4, 5)
		"vmov.32 q15, q3;"			//q15 <- R
		"vzip.32 q15, q3;"			
		"vswp d31, d6;"				//q15<- [R[0], R[0], R[2], R[2]]	q3 <- [R[1], R[1], R[3], R[3]]
		"vmov.32 d9, d10;"			//q4 <- relx, relx, rely, rely

		"vmul.f32 q10, q15, q2;"	//q10<- (dx*R)
		"vmul.f32 q10, q10, q11;"	//q10<- (dx*R) * radius
		"vadd.f32 q9, q10, q4;"		//q9 <- rel[x/y] + (dx*R) * radius
		"vmul.f32 q10, q3, q6;"		//q10<- (dy*R)
		"vmla.f32 q9, q10, q11;"	//q9 <- (rel[x/y] + (dx*R) * radius) + (dy*R) * radius = [x_final, x_final, y_final, y_final]
		//=================================================================================================//

		//------Convert to ushort (Q format)------//
		// "vdup.32 q15, %[Qform6];"	//q15 <- 64
		// "vmul.f32 q7, q7, q15;"
		// "vmul.f32 q7, q7, q15;"
		//----------------------------------------//

		// //===========================================Interpolate===========================================//
		"vdup.32 q11, %[img];"
		"vdup.32 q10, %[width];"

		//-----------------------------------------interpolate q-------------------------------------------//
		
		//--find distances (a1, a2)--//
		"vdup.32 q15, d3[1];"		//q15<- float(1)
		"vmov.u32 q14, #1;"			//q14<- int(1)
		"vcvt.u32.f32 q0, q7;"		//q0 <- floor(x_final) = int(xp)
		"vcvt.f32.u32 q1, q0;"		//q1 <- floor(x_final) = float(xp)
		"vcvt.u32.f32 q2, q8;"		//q2 <- floor(y_final) = int(yp)
		"vcvt.f32.u32 q3, q2;"	 	//q3 <- floor(y_final) = float(yp)

		"vsub.f32 q7, q7, q1;"		//q7 <- x_final - xp = a1
		"vsub.f32 q8, q8, q3;"		//q8 <- y_final - yp = a2

		"vsub.f32 q4, q15, q7;"		//q4 <- (1-a1)
		"vsub.f32 q5, q15, q8;"		//q5 <- (1-a2)
		//----------------------------//

		//-------read first row-------//
		"vmul.u32 q15, q10, q2;"	//q15<- int(yp) * width
		"vadd.u32 q15, q15, q0;"	//q15<- int(yp) * width + int(xp) = imgID1 / 2 (half ID for unsigned short)
		"vshl.u32 q15, q15, #1;"	//q15<- (int(yp) * width + int(xp)) * 2 = imgID1 

		"vadd.u32 q15, q11, q15;"	//q15<- img + imgID1
		"vmov.32 r2, d30[0];"
		"vld1.32 {d26[0]}, [r2];"	//q13<- img[imgID1]
		"vmov.32 r2, d30[1];"
		"vld1.32 {d26[1]}, [r2];"	//q13<- img[imgID1]
		"vmov.32 r2, d31[0];"
		"vld1.32 {d27[0]}, [r2];"	//q13<- img[imgID1]
		"vmov.32 r2, d31[1];"
		"vld1.32 {d27[1]}, [r2];"	//q13<- img[imgID1]

		"vuzp.16 d26, d27;"
		"vmovl.u16 q3, d26;"		//q3 <- img[imgID1]
		"vmovl.u16 q13, d27;"		//q13<- img[imgID1+1]
		//-----------------------------//

		"vdup.32 q1, %[toShort];"	//q1 <- 65536 (float to unsigned char)

		//--find w0--//
		"vmul.f32 q6, q4, q5;"		//q6 <- (1-a1)(1-a2) = w0
		"vmul.f32 q6, q6, q1;"
		"vcvt.u32.f32 q6, q6;"		//q6 <- w0 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmul.u32 q12, q6, q3;"		//q12<- w0*img[imgID1]
		//--------------------------------------//

		//--find w1--//
		"vmul.f32 q6, q7, q5;"		//q6 <- a1 * (1-a2) = w1
		"vmul.f32 q6, q6, q1;"
		"vcvt.u32.f32 q6, q6;"		//q6 <- w1 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 q12, q6, q13;"	//q12<- w0*img[imgID1] + w1*img[imgID1+1]
		//--------------------------------------//

		//-------read second row-------//
		"vadd.u32 q2, q2, q14;"		//q2 <- int(yp) + 1 -> (next row)
		"vmul.u32 q15, q10, q2;"	//q15<- (int(yp) + 1) * width
		"vadd.u32 q15, q15, q0;"	//q15<- (int(yp) + 1) * width + int(xp) = imgID2 / 2 (half ID for unsigned short)
		"vshl.u32 q15, q15, #1;"	//q15<- ((int(yp) + 1) * width + int(xp)) * 2 = imgID2 

		"vadd.u32 q15, q11, q15;"	//q15<- img + imgID2
		"vmov.32 r2, d30[0];"
		"vld1.32 {d26[0]}, [r2];"	//q13<- img[imgID2]
		"vmov.32 r2, d30[1];"
		"vld1.32 {d26[1]}, [r2];"	//q13<- img[imgID2]
		"vmov.32 r2, d31[0];"
		"vld1.32 {d27[0]}, [r2];"	//q13<- img[imgID2]
		"vmov.32 r2, d31[1];"
		"vld1.32 {d27[1]}, [r2];"	//q13<- img[imgID2]

		"vuzp.16 d26, d27;"
		"vmovl.u16 q3, d26;"		//q3 <- img[imgID2]
		"vmovl.u16 q13, d27;"		//q13<- img[imgID2+1]
		//------------------------------//

		//--find w2--//
		"vmul.f32 q6, q4, q8;"		//q6 <- (1-a1) * a2 = w2
		"vmul.f32 q6, q6, q1;"
		"vcvt.u32.f32 q6, q6;"		//q6 <- w2 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 q12, q6, q3;"	//q12<- w0*img[imgID1] + w1*img[imgID1+1] + w2*img[imgID2]
		//--------------------------------------//

		//--find w3--//
		"vmul.f32 q6, q7, q8;"		//q6 <- a1 * a2 = w3
		"vmul.f32 q6, q6, q1;"
		"vcvt.u32.f32 q6, q6;"		//q6 <- w3 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 q12, q6, q13;"	//q12<- w0*img[imgID1] + w1*img[imgID1+1] + w2*img[imgID2] + w3*img[imgID2+1]
		//--------------------------------------//

		"vrshrn.u32 d24, q12, #16;" //d24<-  unsigned short(interpolated value)
		"vst1.16 {d24}, [%[samples]]!;"
		//-------------------------------------------------------------------------------------------------//

		//-----------------------------------------interpolate d-------------------------------------------//

		//--find distances (a1, a2)--//
		"vmov.f32 s0, #3212836864;"	//s0=-1.0
		"vmul.f32 q0, q0, q0;"		//s0=1.0
		"vdup.32 d30, d0[0];"		//d30<- float(1)
		"vmov.u32 d28, #1;"			//d28<- int(1)//------------------------------------------(not needed!!! (test that))
		"vcvt.u32.f32 d0, d18;"		//d0 <- floor(x_final) = int(xp)
		"vcvt.f32.u32 d2, d0;"		//d2 <- floor(x_final) = float(xp)
		"vcvt.u32.f32 d4, d19;"		//d4 <- floor(y_final) = int(yp)
		"vcvt.f32.u32 d6, d4;"	 	//d6 <- floor(y_final) = float(yp)

		"vsub.f32 d14, d18, d2;"	//d14 <- x_final - xp = a1
		"vsub.f32 d16, d19, d6;"	//d16 <- y_final - yp = a2

		"vsub.f32 d8, d30, d14;"	//d8 <- (1-a1)
		"vsub.f32 d10, d30, d16;"	//d10<- (1-a2)
		//----------------------------//

		//-------read first row-------//
		"vmul.u32 d30, d20, d4;"	//q30<- int(yp) * width
		"vadd.u32 d30, d30, d0;"	//d30<- int(yp) * width + int(xp) = imgID1 / 2 (half ID for unsigned short)
		"vshl.u32 d30, d30, #1;"	//d30<- (int(yp) * width + int(xp)) * 2 = imgID1 

		"vadd.u32 d30, d22, d30;"	//d30<- img + imgID1
		"vmov.32 r2, d30[0];"
		"vld1.32 {d26[0]}, [r2];"	//q13<- img[imgID1]
		"vmov.32 r2, d30[1];"
		"vld1.32 {d26[1]}, [r2];"	//q13<- img[imgID1]

		"vrev32.16 d27, d26;"
		"vuzp.16 d26, d26;"
		"vuzp.16 d27, d27;"
		"vmovl.u16 q3, d26;"		//d6 <- img[imgID1]
		"vmovl.u16 q13, d27;"		//d26<- img[imgID1+1]
		//-----------------------------//   (good shape)

		"vdup.32 d2, %[toShort];"	//d2 <- 65536 (float to unsigned char)

		//--find w0--//
		"vmul.f32 d12, d8, d10;"	//d12<- (1-a1)(1-a2) = w0
		"vmul.f32 d12, d12, d2;"
		"vcvt.u32.f32 d12, d12;"	//d12<- w0 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmul.u32 d24, d12, d6;"	//d24<- w0*img[imgID1]
		//--------------------------------------//

		//--find w1--//
		"vmul.f32 d12, d14, d10;"	//d12<- a1 * (1-a2) = w1
		"vmul.f32 d12, d12, d2;"
		"vcvt.u32.f32 d12, d12;"	//d12 <- w1 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 d24, d12, d26;"	//d24<- w0*img[imgID1] + w1*img[imgID1+1]
		//--------------------------------------//

		//-------read second row-------//
		"vadd.u32 d4, d4, d28;"		//d4 <- int(yp) + 1 -> (next row)
		"vmul.u32 d30, d20, d4;"	//d30<- (int(yp) + 1) * width
		"vadd.u32 d30, d30, d0;"	//d30<- (int(yp) + 1) * width + int(xp) = imgID2 / 2 (half ID for unsigned short)
		"vshl.u32 d30, d30, #1;"	//d30<- ((int(yp) + 1) * width + int(xp)) * 2 = imgID2 

		"vadd.u32 d30, d22, d30;"	//d30<- img + imgID2
		"vmov.32 r2, d30[0];"
		"vld1.32 {d26[0]}, [r2];"	//q13<- img[imgID2]
		"vmov.32 r2, d30[1];"
		"vld1.32 {d26[1]}, [r2];"	//q13<- img[imgID2]

		"vrev32.16 d27, d26;"
		"vuzp.16 d26, d26;"
		"vuzp.16 d27, d27;"
		"vmovl.u16 q3, d26;"		//d6 <- img[imgID2]
		"vmovl.u16 q13, d27;"		//d26<- img[imgID2+1]
		//-----------------------------//

		//--find w2--//
		"vmul.f32 d12, d8, d16;"	//d12 <- (1-a1) * a2 = w2
		"vmul.f32 d12, d12, d2;"
		"vcvt.u32.f32 d12, d12;"	//d12 <- w2 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 d24, d12, d6;"	//d24<- w0*img[imgID1] + w1*img[imgID1+1] + w2*img[imgID2]
		//--------------------------------------//

		//--find w3--//
		"vmul.f32 d12, d14, d16;"	//d12 <- a1 * a2 = w3
		"vmul.f32 d12, d12, d2;"
		"vcvt.u32.f32 d12, d12;"	//d12 <- w3 ->[0...65536]
		//-----------//

		//---accumulate to interpolated value---//
		"vmla.u32 d24, d12, d26;"	//d24<- w0*img[imgID1] + w1*img[imgID1+1] + w2*img[imgID2] + w3*img[imgID2+1]
		//--------------------------------------//

		"vrshr.u32 d24, d24, #16;"  //d24<-  unsigned short(interpolated value)
		"vuzp.16 d24, d24;"
		"vst1.16 {d24}, [%[samples]]!;"
		// -------------------------------------------------------------------------------------------------//

		// =================================================================================================//
		: 
		: [samples] "r" (samples), [dx] "r" (dx), [dy] "r" (dy), [R] "r" (R), [relx] "r" (relx), [rely] "r" (rely),
		[radius] "r" (radius), [img] "r" (img), [width] "r" (width), [toShort] "r" (toShort)
		: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "q14", "q15", "r2"
	);


}

float interpolate_pixel_bilinear_Neon(const cv::Mat &image, float x, float y)
{
	int xp, yp;
	int xp_plus_1, yp_plus_1;
	float w0, w1, w2, w3;
	const unsigned short* p0;
	const unsigned short* p1;
	float result;

	xp = (int) x;
	yp = (int) y;

	xp_plus_1 = xp+1;
	yp_plus_1 = yp+1;

	p0 = image.ptr<unsigned short>(yp);
	assert(p0);
	p1 = image.ptr<unsigned short>(yp_plus_1);
	assert(p1);

	w0 = (xp_plus_1 - x)  * (yp_plus_1 - y);
	w1 = (x         - xp) * (yp_plus_1 - y);
	w2 = (xp_plus_1 - x)  * (y         - yp);
	w3 = (x         - xp) * (y         - yp);

	result = w0*p0[xp] + w1*p0[xp_plus_1] + w2*p1[xp] + w3*p1[xp_plus_1];

  return result;

}

#endif