/*
 * lkf.c
 *
 *  Created on: Apr 4, 2025
 *      Author: Lenovo310
 */
#include <assert.h>
#include "arm_math.h"
#include "lkf.h"

#define ARM_MATH_CM4

typedef struct kf
{
	/* state vector */
	arm_matrix_instance_f32 x;

	/* covariance vector */
	arm_matrix_instance_f32 P;

	/* state transition matrix */
	arm_matrix_instance_f32 F;

	/* measurement sensitivity matrix*/
	arm_matrix_instance_f32 H;

	/* z measurement vector */
	arm_matrix_instance_f32 z;

	/* y_ innovation vector */
	arm_matrix_instance_f32 y_;

	/* S innovation covariance */
	arm_matrix_instance_f32 S;

	/* Q prediction covariance */
	arm_matrix_instance_f32 Q;

	/* R measurement covariance */
	arm_matrix_instance_f32 R;

	/* K Kalman Gain matrix */
	arm_matrix_instance_f32 K;

} Lkf;

Lkf LinearKalman;

arm_matrix_instance_f32 Temp2x2;
arm_matrix_instance_f32 Ft;

arm_matrix_instance_f32 Temp2x1;

static float Temp2x2_[2][2] =
{
	{0.0, 0.0},
	{0.0, 0.0},
};

static float Ft_[2][2] =
{
	{0.0, 0.0},
	{0.0, 0.0},
};

//static float Temp2x1_[2] =
//{
//	0.0, 0.0
//};

#if 0
static float x_[4][1] =
{
	0.0, 0.0, 0.0, 0.0
};

static float z_[4][1] =
{
	0.0, 0.0, 0.0, 0.0
};

static float F_[4][4] =
{
	{1.0, 0.0, 0.0, 0.0},
	{0.0, 1.0, 0.0, 0.0},
	{0.0, 0.0, 1.0, 0.0},
	{0.0, 0.0, 0.0, 1.0}
};

static float H_[4][4] =
{
	{1.0, 0.0, 0.0, 0.0},
	{0.0, 1.0, 0.0, 0.0},
	{0.0, 0.0, 1.0, 0.0},
	{0.0, 0.0, 0.0, 1.0}
};

static float Q_[2][2] =
{
	{0.5, 0.0},
	{0.0, 0.5}
};

static float R_[4][4] =
{
	{5.0, 0.0, 0.0, 0.0},
	{0.0, 5.0, 0.0, 0.0},
	{0.0, 0.0, 10.0, 0.0},
	{0.0, 0.0, 0.0, 10.0},
};
#endif

// state vector is [pitch, pitch_rate]^T
static float x_[2] =
{
	1.0, 1.0
};

static float z_[2] =
{
	0.0, 0.0
};

static float y_[2] =
{
	0.0, 0.0
};

static float P_[2][2] =
{
	{1.0, 0.0},
	{0.0, 1.0},
};

static float F_[2][2] =
{
	{1.0, 0.0},
	{0.0, 1.0},
};

static float H_[2][2] =
{
	{1.0, 0.0},
	{0.0, 1.0},
};

static float Q_[2][2] =
{
	{5.0, 0.0},
	{0.0, 5.0}
};

static float R_[2][2] =
{
	{5.0, 0.0},
	{0.0, 10.0},
};

static float S_[2][2] =
{
	{0.0, 0.0},
	{0.0, 0.0},
};

static float K_[2][2] =
{
	{0.0, 0.0},
	{0.0, 0.0},
};

static inline void lkf_prediction (void)
{
	arm_status ret;

	// Predict x- from previous x using F
	ret = arm_mat_mult_f32(&LinearKalman.F, &LinearKalman.x, &LinearKalman.x);
	assert(ret == ARM_MATH_SUCCESS);

	// Predict covariance P- from previous P using F and Q
	// P- = F*P*FT + Q
	// Break this up into 3 ops since cmsis matrix ops can only take 2 args at a time

	// temp = F * P
	ret = arm_mat_mult_f32(&LinearKalman.F, &LinearKalman.P, &Temp2x2);
	assert(ret == ARM_MATH_SUCCESS);

	// P = temp * Ft
	ret = arm_mat_mult_f32(&Temp2x2, &Ft, &LinearKalman.P);
	assert(ret == ARM_MATH_SUCCESS);

    // P += Q
	ret = arm_mat_add_f32(&LinearKalman.P, &LinearKalman.Q, &LinearKalman.P);
	assert(ret == ARM_MATH_SUCCESS);
}

static inline void lkf_correction (float *measured_vals, int measurement_len)
{
	arm_status ret;

	z_[0] = measured_vals[0];
	z_[1] = measured_vals[1];

	// Compute innovation vector
	ret = arm_mat_sub_f32(&LinearKalman.z, &LinearKalman.x, &LinearKalman.y_);
	assert(ret == ARM_MATH_SUCCESS);

	// Compute innovation covariance
	ret = arm_mat_add_f32(&LinearKalman.P, &LinearKalman.R, &LinearKalman.S);
	assert(ret == ARM_MATH_SUCCESS);

	/*
	 * Compute Kalman gain
	 * K = P- * S^-1
	 * Break this into 2 ops
	 */

	// temp = S inverse
	ret = arm_mat_inverse_f32(&LinearKalman.S, &Temp2x2);
	assert(ret == ARM_MATH_SUCCESS);

	// K = P- * temp
    ret = arm_mat_mult_f32(&LinearKalman.P, &Temp2x2, &LinearKalman.K);
	assert(ret == ARM_MATH_SUCCESS);

	/* Apply corrections to x- => x and P- => P */

	// x^ = x- + K*y_
	// Break this into 2 ops

	// temp = K*y_
	ret = arm_mat_mult_f32(&LinearKalman.K, &LinearKalman.y_, &Temp2x2);
	assert(ret == ARM_MATH_SUCCESS);

	// x^ = x- + temp
	ret = arm_mat_add_f32(&LinearKalman.x, &Temp2x2, &LinearKalman.x);
	assert(ret == ARM_MATH_SUCCESS);

	// P = (I - K * H) * P-
	// H is identity matrix itself, so P = (H - K) * P-
	ret = arm_mat_sub_f32(&LinearKalman.H, &LinearKalman.K, &Temp2x2);
	assert(ret == ARM_MATH_SUCCESS);

	ret = arm_mat_mult_f32(&Temp2x2, &LinearKalman.P, &LinearKalman.P);
	assert(ret == ARM_MATH_SUCCESS);


}

void lkf_update (float *measured_vals, int measurement_len)
{

	lkf_prediction();

	lkf_correction(measured_vals, measurement_len);
}

void lkf_init (float dt)
{
	F_[0][2] = dt;

	// TODO: Initial values for state vector
	arm_mat_init_f32(&LinearKalman.x, 2, 1, &x_[0]);

	arm_mat_init_f32(&LinearKalman.F, 2, 2, &F_[0][0]);

	arm_mat_init_f32(&LinearKalman.H, 2, 2, &H_[0][0]);

	arm_mat_init_f32(&LinearKalman.z, 2, 1, &z_[0]);

	arm_mat_init_f32(&LinearKalman.y_, 2, 1, &y_[0]);

	arm_mat_init_f32(&LinearKalman.Q, 2, 2, &Q_[0][0]);

	arm_mat_init_f32(&LinearKalman.R, 2, 2, &R_[0][0]);

	arm_mat_init_f32(&LinearKalman.S, 2, 2, &S_[0][0]);

	arm_mat_init_f32(&LinearKalman.K, 2, 2, &K_[0][0]);

	arm_mat_init_f32(&LinearKalman.P, 2, 2, &P_[0][0]);

	arm_mat_init_f32(&Temp2x2, 2, 2, &Temp2x2_[0][0]);
	arm_mat_init_f32(&Ft, 2, 2, &Ft_[0][0]);

	//arm_mat_init_f32(&Temp2x1, 2, 1, &Temp2x1_[0]);

	arm_status ret = arm_mat_trans_f32(&LinearKalman.F, &Ft);
	assert(ret == ARM_MATH_SUCCESS);
}
