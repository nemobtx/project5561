#ifndef FREAK_NEON
#define FREAK_NEON


void freakExtractorNeon(unsigned char *my_descr, unsigned short *samples, unsigned char *oper, unsigned short *andPattern)
{

	unsigned short center = samples[0];
	samples = samples + 1;
	asm volatile(

		"vld1.16 {q6}, [%[andPattern]];"
		"vshl.u16 q7, q6, #8;"
		"vld1.16 {q9}, [%[samples]]!;"	 //load samples[8] ...samples[15]
		"vld1.16 {q10}, [%[samples]]!;"  //load samples[16]...samples[23]
		"vld1.16 {q11}, [%[samples]]!;"  //load samples[24]...samples[31]
		"vld1.16 {q12}, [%[samples]]!;"  //load samples[32]...samples[39]
		"vld1.16 {q13}, [%[samples]]!;"  //load samples[40]...samples[47]
		"vld1.16 {q14}, [%[samples]]!;"  //load samples[48], samples[49], ..garbages..
		"vmov.u8 d29, #1;"
		"vdup.16 q0, %[center];"

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		//compare center with everything
		"vcgt.u16 q1, q9, q0;"  	// center < samples [8,9,10,11,12,13,14,15]
		"vcgt.u16 q2, q10, q0;" 	// center < samples [16,17,18,19,20,21,22,23]

		"vand.i16 q1, q1, q6;"
		"vand.i16 q2, q2, q7;"
		"vorr.i16 q1, q1, q2;"
		"vorr.i16 d2, d2, d3;"

		"vcgt.u16 q3, q11, q0;"  	// center < samples [24,25,26,27,28,29,30,31]
		"vcgt.u16 q4, q12, q0;"  	// center < samples [32,33,34,35,36,37,38,39]

		"vand.i16 q3, q3, q6;"
		"vand.i16 q4, q4, q7;"
		"vorr.i16 q3, q3, q4;"
		"vorr.i16 d3, d6, d7;"

		"vshl.u64 q2, q1, #32;"
		"vorr.i16 q1, q1, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;"

		"vcgt.u16 q2, q13, q0;"		// q2 <- center < samples [40,41,42,43,44,45,46,47]

		//compare every pair fo receptors that are in the same chunk

		"vdup.16 d6, d18[0];"		// d6 <- sample[8]
		"vext.8 d0, d0, d6, #4;"    // q0 <- cntr, cntr, smpl[8], smpl[8]
		"vmov d1, d6;"				// q0 <- cntr, cntr, smpl[8], smpl[8], smpl[8], smpl[8], smpl[8], smpl[8]
		"vdup.16 d30, d18[1];"		// d30<- sample[8+1]
		"vext.8 d1, d1, d30, #2;"   // q0 <- cntr, cntr, smpl[8], smpl[8], smpl[8], smpl[8], smpl[8], smpl[8+1]

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vtbx.8 d2, {q14}, d30;"
		"vtbx.8 d2, {q9}, d31;"		// d2 <- smpl[48], smpl[49], smpl[8 + 1], smpl[8 + 2]

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vtbx.8 d3, {q9}, d30;"		// q1 <- smpl[48], smpl[49], smpl[8+1], smpl[8+2], smpl[8+3], smpl[8+4], smpl[8+5], smpl[8+2]

		"vcgt.u16 q3, q1, q0;"  	// q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		// d8 <- ready 

		"vtbx.8 d0, {q9}, d31;" 	//d0  <- smpl[8+1], smpl[8+1], smpl[8+1], smpl[8+2]

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q9}, d30;"		//q0  <- smpl[8+1], smpl[8+1], smpl[8+1], smpl[8+2], smpl[8+2], smpl[8+2], smpl[8+3], smpl[8+3]
		"vtbx.8 d2, {q9}, d31;"		//d2  <- smpl[8+3], smpl[8+4], smpl[8+5], smpl[8+3]

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q9}, d30;"		//q1  <- smpl[8+3], smpl[8+4], smpl[8+5], smpl[8+3], smpl[8+4], smpl[8+5], smpl[8+4], smpl[8+5] 
		"vcgt.u16 q2, q1, q0;"  	//q2 <- q0 < q1

		"vtbx.8 d0, {q9}, d31;"		//d0 <-  smpl[8+4], smpl[16], smpl[16], smpl[16]

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q9}, d30;"		//q0 <-  smpl[8+4], smpl[16], smpl[16], smpl[16], smpl[16], smpl[16], smpl[16+1], smpl[16+1]
		"vtbx.8 d2, {q9}, d31;"		//d2 <-  smpl[8+5], smpl[16+1]

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q10}, d30;"	//d2 <-  smpl[8+5], smpl[16+1], smpl[16+2], smpl[16+3]
		"vtbx.8 d3, {q10}, d31;"	//q1 <-  smpl[8+5], smpl[16+1], smpl[16+2], smpl[16+3], smpl[8+4], smpl[16+5], smpl[16+2], smpl[16+3]

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		// d9 <- ready 

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d0, {q10}, d31;"	//d0 <- smpl[16+1], smpl[16+1], smpl[16+2], smpl[16+2]

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q10}, d30;"	//q0 <- smpl[16+1], smpl[16+1], smpl[16+2], smpl[16+2], smpl[16+2], smpl[16+3], smpl[16+3], smpl[16+4]
		"vtbx.8 d2, {q10}, d31;"	//d2 <- smpl[16+4], smpl[16+5], smpl[16+3], smpl[16+4]

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q10}, d30;"	//q1 <- smpl[16+5], smpl[16+4], smpl[16+5], smpl[16+5], smpl[16+4], smpl[16+5], smpl[16+3], smpl[16+4]
		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q10}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vmov d6, d21;"
		"vmov d7, d22;" 			//q3 <- d22, d21

		"vtbx.8 d1, {q10}, d30;"
		"vtbx.8 d2, {q3}, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q3}, d30;"
		"vcgt.u16 q4, q1, q0;"		//q4 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q4, q4, q7;"
		"vorr.i16 q2, q4, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vtbx.8 d0, {q10}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q10}, d30;"
		"vtbx.8 d1, {q11}, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q3}, d30;"
		"vtbx.8 d3, {q3}, d31;"
		"vcgt.u16 q2, q1, q0;"    	//q2 <- q0 < q1

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q11}, d30;"
		"vtbx.8 d1, {q11}, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q11}, d30;"
		"vtbx.8 d3, {q11}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		// d9 <- ready 

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q11}, d30;"
		"vtbx.8 d1, {q11}, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q12}, d30;"
		"vtbx.8 d2, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q11}, d30;"
		"vtbx.8 d3, {q12}, d31;"

		
		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q12}, d30;"
		"vtbx.8 d1, {q12}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q12}, d30;"
		"vtbx.8 d3, {q12}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q12}, d30;"
		"vtbx.8 d1, {q12}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q12}, d30;"
		"vtbx.8 d3, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q13}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q12}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d1, {q12}, d30;"
		"vtbx.8 d1, {q13}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q13}, d30;"
		"vtbx.8 d3, {q13}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" // 16 unsigned char ready

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q13}, d30;"
		"vtbx.8 d1, {q13}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vext.8 q4, q13, q14, #4;"

		"vtbx.8 d2, {q13}, d30;"
		"vtbx.8 d3, {q4}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q13}, d30;"
		"vtbx.8 d1, {q13}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q4}, d30;"
		"vtbx.8 d3, {q4}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q4}, d30;"		//d2 <- smpl[56+4], smpl[56+5], smpl[56+5], -

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q13}, d31;"

		"vext.8 d0, d0, d28, #4;"  //d0 <- smpl[56+3], smpl[56+3], smpl[56+4], -

		//compare every pair of receptrors that are in neighbouring chunks
		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		
		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vmov d16, d19;"
		"vmov d17, d20;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"


		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;"

		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vmov d16, d21;"
		"vmov d17, d22;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d1, {q9}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(6th group of 4 unsigned chars)

		"vtbx.8 d0, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q10}, d30;"
		"vtbx.8 d1, {q10}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"

		
		"vtbx.8 d0, {q10}, d30;"
		"vtbx.8 d1, {q10}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d0, {q10}, d30;"
		"vtbx.8 d1, {q10}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q10}, d30;"
		"vtbx.8 d1, {q10}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d2, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q11}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(7th group of 4 unsigned chars)

		"vtbx.8 d0, {q10}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q10}, d30;"
		"vtbx.8 d2, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q11}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q10}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q10}, d30;"
		"vtbx.8 d2, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d3, {q11}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vtbx.8 d0, {q10}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q11}, d30;"
		"vtbx.8 d1, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q11}, d30;"
		"vtbx.8 d3, {q11}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q11}, d30;"
		"vtbx.8 d1, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q11}, d30;"
		"vtbx.8 d3, {q11}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q12}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(8th group of 4 unsigned chars)

		"vtbx.8 d0, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d1, {q11}, d30;"
		"vtbx.8 d2, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q12}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d1, {q11}, d30;"
		"vtbx.8 d2, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q12}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vtbx.8 d0, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d1, {q11}, d30;"
		"vtbx.8 d2, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q12}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d1, {q11}, d30;"
		"vtbx.8 d2, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q12}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(9th group of 4 unsigned chars)

		"vtbx.8 d0, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q12}, d30;"
		"vtbx.8 d1, {q12}, d31;"

		"vmov d17, d26;"
		"vmov d16, d25;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q12}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"
		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1
		"vtbx.8 d0, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q12}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"
		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vtbx.8 d0, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q12}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"
		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1
		"vtbx.8 d0, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q12}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(10th group of 4 unsigned chars)

		"vtbx.8 d0, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q12}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"

		"vmov d17, d28;"
		"vmov d16, d27;"

		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q12}, d30;"
		"vtbx.8 d1, {q12}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vtbx.8 d0, {q12}, d30;"
		"vtbx.8 d0, {q13}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d1, {q13}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q13}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d1, {q13}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q8}, d30;"
		
		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(11th group of 4 unsigned chars)

		"vtbx.8 d0, {q13}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d1, {q13}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q13}, d31;"    //d0 <- smpl[48+5], smpl[48+5], smpl[48+5], -

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q8}, d30;"		//d2 <- smpl[56+3], smpl[56+4], smpl[56+5], -

		//compare all pairs along each line throut the center
		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d0, {q9}, d31;"		//d0 <- smpl[48+5], smpl[48+5], smpl[48+5], smpl[8+0]

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vmov d16, d21;"
		"vmov d17, d22;"

		"vtbx.8 d2, {q8}, d30;"		//d2 <- smpl[56+3], smpl[56+4], smpl[56+5], smpl[24+0]
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vtbx.8 d0, {q9}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q9}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"
		"vtbx.8 d3, {q12}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q12}, d30;"
		"vtbx.8 d3, {q12}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(12th group of 4 unsigned chars)

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d2, {q12}, d30;"

		"vmov d16, d27;"
		"vmov d17, d28;"

		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q9}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d1, {q9}, d30;"

		"vmov d18, d19;"
		"vmov d19, d20;"

		"vtbx.8 d1, {q9}, d31;"   //209

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q11}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vtbx.8 d0, {q9}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q9}, d30;"
		"vtbx.8 d2, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q11}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q9}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q9}, d30;"
		"vtbx.8 d2, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vmov d16, d25;"
		"vmov d17, d26;"

		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(13th group of 4 unsigned chars) (221)

		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vmov d18, d21;"
		"vmov d19, d22;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d1, {q9}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q8}, d30;"
		"vtbx.8 d3, {q12}, d31;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q12}, d30;"
		"vtbx.8 d3, {q12}, d31;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q9}, d30;"
		"vtbx.8 d1, {q9}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q12}, d30;"

		"vmov d16, d27;"
		"vmov d17, d28;"

		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q9}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d1, {q9}, d30;"
		"vtbx.8 d1, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		"pld 	[%[oper2]];"


		"vtbx.8 d2, {q8}, d30;"
		"vtbx.8 d3, {q8}, d31;"

		"vmov d20, d25;"
		"vmov d21, d26;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q10}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(14th group of 4 unsigned chars) (242)

		"vtbx.8 d0, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d1, {q11}, d30;"
		"vtbx.8 d2, {q10}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d3, {q10}, d30;"

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vtbx.8 d0, {q11}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d0, {q12}, d30;"
		"vtbx.8 d1, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbx.8 d2, {q10}, d30;"
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"
		"vld1.u8 {q5}, [%[oper2]]!;"
		// "pld 	[%[oper2]];"


		"vtbx.8 d3, {q8}, d30;"

		"vcgt.u16 q3, q1, q0;"		//q3 <- q0 < q1

		"vand.i16 q2, q2, q6;"
		"vand.i16 q3, q3, q7;"
		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d8, d4, d5;"		//d8 <- ready

		"vtbx.8 d0, {q12}, d31;"

		"vshl.u8 d30, d10, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbl.8 d1, {q12}, d30;"  //last one must be vtbl
		"vtbx.8 d2, {q8}, d31;"

		"vshl.u8 d30, d11, #1;"
		"vadd.u8 d31, d30, d29;"
		"vzip.8 d30, d31;"

		"vtbl.8 d3, {q8}, d30;"	  //last one must be vtbl

		"vcgt.u16 q2, q1, q0;"		//q2 <- q0 < q1

		"vmov.u8 q3, #0;"
		
		"vand.i16 q2, q2, q6;"

		"vorr.i16 q2, q3, q2;"
		"vorr.i16 d9, d4, d5;"		//d9 <- ready

		"vshl.u64 q2, q4, #32;"
		"vorr.i16 q1, q4, q2;"
		"vshl.i32 q2, q1, #16;"
		"vorr.i16 q1, q1, q2;"

		"vst2.16 {d2[3],d3[3]}, [%[my_descr]]!;" //(15th group of 4 unsigned chars) (242)

		"vst2.16 {d6[0],d7[0]}, [%[my_descr]];"

		: 
		: [samples] "r" (samples), [my_descr] "r" (my_descr), [andPattern] "r" (andPattern), [center] "r" (center), [oper2] "r" (oper)
		: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "q14", "q15", "memory"
	);
}
#endif