/*
 * lkf.h
 *
 *  Created on: Apr 4, 2025
 *      Author: Lenovo310
 */

#ifndef INC_LKF_H_
#define INC_LKF_H_

void lkf_update (float *measured_vals, int measurement_len);
void lkf_init (float dt);

#endif /* INC_LKF_H_ */
