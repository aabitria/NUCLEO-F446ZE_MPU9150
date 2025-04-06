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
#define STATE_VEC_LEN		(4)
#define SENSE_VEC_LEN		(4)

/*  IMU State vector =
 *  |            |
 *  | pitch      |
 *  | roll       |
 *  | pitch_rate |
 *  | roll_rate  |
 *  |            |
 */

typedef struct kf
{
	/*
	 * Vectors will not need an arm_mat instance,
	 * it can be multiplied with a matrix with
	 * a vec_mult cmsis-dsp operation
	 */
	/* state vector */
	float x[STATE_VEC_LEN];

	/* z measurement vector */
	float z[SENSE_VEC_LEN];

	/* y_ innovation vector */
	float y_[SENSE_VEC_LEN];

	/* covariance matrix */
	arm_matrix_instance_f32 P;

	/* state transition matrix */
	arm_matrix_instance_f32 F;

	/* measurement sensitivity matrix*/
	arm_matrix_instance_f32 H;

	/* S innovation covariance */
	arm_matrix_instance_f32 S;

	/* Q process noise covariance */
	arm_matrix_instance_f32 Q;

	/* R measurement covariance */
	arm_matrix_instance_f32 R;

	/* K Kalman Gain matrix */
	arm_matrix_instance_f32 K;

} Lkf;

Lkf LinearKalman;

arm_matrix_instance_f32 Temp;
arm_matrix_instance_f32 Tmp;
arm_matrix_instance_f32 Ft;


static float Tmp_[4] =
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

/* can also be used as Identity matrix */
static float H_[4][4] =
{
	{1.0, 0.0, 0.0, 0.0},
	{0.0, 1.0, 0.0, 0.0},
	{0.0, 0.0, 1.0, 0.0},
	{0.0, 0.0, 0.0, 1.0}
};

static float Q_[4][4] =
{
	{0.01, 0.0, 0.0, 0.0},
	{0.0, 0.01, 0.0, 0.0},
	{0.0, 0.0, 0.01, 0.0},
	{0.0, 0.0, 0.0, 0.01},
};

static float R_[4][4] =
{
	{10.0, 0.0, 0.0, 0.0},
	{0.0, 10.0, 0.0, 0.0},
	{0.0, 0.0, 10.0, 0.0},
	{0.0, 0.0, 0.0, 10.0},
};

static float P_[4][4] =
{
	{1.0, 0.0, 0.0, 0.0},
	{0.0, 1.0, 0.0, 0.0},
	{0.0, 0.0, 1.0, 0.0},
	{0.0, 0.0, 0.0, 1.0}
};

static float S_[4][4] =
{
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0}
};

static float K_[4][4] =
{
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0}
};

static float Ft_[4][4] =
{
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0},
};

static float Temp_[4][4] =
{
	{1.0, 0.0, 0.0, 0.0},
	{0.0, 1.0, 0.0, 0.0},
	{0.0, 0.0, 1.0, 0.0},
	{0.0, 0.0, 0.0, 1.0}
};


static inline void lkf_prediction (void)
{
	arm_status ret;

	// Predict x- from previous x using F
	arm_mat_vec_mult_f32(&LinearKalman.F, LinearKalman.x, LinearKalman.x);

	// Predict covariance P- from previous P using F and Q
	// P- = F*P*FT + Q
	// Break this up into 3 ops since cmsis matrix ops can only take 2 args at a time

	// temp = F * P
	ret = arm_mat_mult_f32(&LinearKalman.F, &LinearKalman.P, &Temp);
	assert(ret == ARM_MATH_SUCCESS);

	// P = temp * Ft
	ret = arm_mat_mult_f32(&Temp, &Ft, &LinearKalman.P);
	assert(ret == ARM_MATH_SUCCESS);

    // P += Q
	ret = arm_mat_add_f32(&LinearKalman.P, &LinearKalman.Q, &LinearKalman.P);
	assert(ret == ARM_MATH_SUCCESS);
}

static inline void lkf_correction (float *measured_vals, int measurement_len)
{
	arm_status ret;

	LinearKalman.z[0] = measured_vals[0];   // angle about x-axis
	LinearKalman.z[1] = measured_vals[1];   // angle about y-axis
	LinearKalman.z[2] = measured_vals[2];   // angular velocity about x-axis
	LinearKalman.z[3] = measured_vals[3];   // angular velocity about y-axis

	// Compute innovation vector
	// y_ = z - H *x, but H is I, so y_ = z - x
	LinearKalman.y_[0] = LinearKalman.z[0] - LinearKalman.x[0];
	LinearKalman.y_[1] = LinearKalman.z[1] - LinearKalman.x[1];
	LinearKalman.y_[2] = LinearKalman.z[2] - LinearKalman.x[2];
	LinearKalman.y_[3] = LinearKalman.z[3] - LinearKalman.x[3];

	// Compute innovation covariance
	// S = P + R
	ret = arm_mat_add_f32(&LinearKalman.P, &LinearKalman.R, &LinearKalman.S);
	assert(ret == ARM_MATH_SUCCESS);

	/*
	 * Compute Kalman gain
	 * K = P- * S^-1
	 * Break this into 2 ops
	 */

	// temp = S inverse
	ret = arm_mat_inverse_f32(&LinearKalman.S, &Temp);
	assert(ret == ARM_MATH_SUCCESS);

	// K = P- * temp
    ret = arm_mat_mult_f32(&LinearKalman.P, &Temp, &LinearKalman.K);
	assert(ret == ARM_MATH_SUCCESS);

	/* Apply corrections to x- => x and P- => P */

	// x^ = x- + K*y_
	// Break this into 2 ops

	// temp = K*y_
	arm_mat_vec_mult_f32(&LinearKalman.K, LinearKalman.y_, Tmp_);

	// x^ = x- + temp
	LinearKalman.x[0] += Tmp_[0];
	LinearKalman.x[1] += Tmp_[1];
	LinearKalman.x[2] += Tmp_[2];
	LinearKalman.x[3] += Tmp_[3];

	// P = (I - K * H) * P-
	// H is identity matrix itself, so P = (H - K) * P-
	ret = arm_mat_sub_f32(&LinearKalman.H, &LinearKalman.K, &Temp);
	assert(ret == ARM_MATH_SUCCESS);

	ret = arm_mat_mult_f32(&Temp, &LinearKalman.P, &LinearKalman.P);
	assert(ret == ARM_MATH_SUCCESS);
}

void lkf_update (float *measured_vals, int measurement_len)
{
    assert(measurement_len == STATE_VEC_LEN);

	lkf_prediction();

	lkf_correction(measured_vals, measurement_len);
}

static void lkf_init_vectors (void)
{
	int i = 0;

	for (i = 0; i < STATE_VEC_LEN; i++)
	{
		LinearKalman.x[i] = 0.0f;
	}

	for (i = 0; i < SENSE_VEC_LEN; i++)
	{
		LinearKalman.z[i] = 0.0f;
	}

	for (i = 0; i < SENSE_VEC_LEN; i++)
	{
		LinearKalman.y_[i] = 0.0f;
	}
}

void lkf_init (float dt)
{
	lkf_init_vectors();

	F_[0][2] = dt;
	F_[1][3] = dt;

	arm_mat_init_f32(&Tmp, 4, 1, &Tmp_[0]);

	arm_mat_init_f32(&LinearKalman.F, 4, 4, &F_[0][0]);
	arm_mat_init_f32(&LinearKalman.H, 4, 4, &H_[0][0]);
	arm_mat_init_f32(&LinearKalman.P, 4, 4, &P_[0][0]);

	arm_mat_init_f32(&LinearKalman.Q, 4, 4, &Q_[0][0]);
	arm_mat_init_f32(&LinearKalman.R, 4, 4, &R_[0][0]);
	arm_mat_init_f32(&LinearKalman.S, 4, 4, &S_[0][0]);
	arm_mat_init_f32(&LinearKalman.K, 4, 4, &K_[0][0]);

	/* Create the transpose of F */
	arm_mat_init_f32(&Ft, 4, 4, &Ft_[0][0]);
	arm_status ret = arm_mat_trans_f32(&LinearKalman.F, &Ft);
	assert(ret == ARM_MATH_SUCCESS);

	arm_mat_init_f32(&Temp, 4, 4, &Temp_[0][0]);
}
