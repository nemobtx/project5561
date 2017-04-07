/*
 * gradients.h
 *
 *  Created on: Mar 30, 2015
 *      Author: mars
 */

#ifndef SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_GRADIENTS_H_
#define SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_GRADIENTS_H_

#include <math.h>
#include <string.h>

void get_gradients(unsigned char *img, double *Gx, double *Gy, double *G, int w, int h);


#endif /* SUBPROJECTS__MARSFRAMEWORK_LIBRARIES_EYEMARS_EYEMARS_INCLUDE_IMGPROC_DENSE_GRADIENTS_H_ */
