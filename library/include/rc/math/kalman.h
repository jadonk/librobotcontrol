/**
 * <rc/math/kalman.h>
 *
 * @brief      Kalman filter implementation
 *
 *             This may be used to implement a discrete time linear or extended
 *             kalman filter.
 *
 *             For the linear case, initialize the filter with
 *             rc_kalman_alloc_lin() which takes in the linear state matrices.
 *             These are then saved and used by rc_kalman_update_lin to
 *             calculate the predicted state x_pre and predicted sensor
 *             measurements h internally each step.
 *
 *
 *             Basic loop structure for Linear case:
 *
 *             - rc_kalman_t kf = rc_kalman_empty();
 *             - rc_kalman_alloc_lin(&kf,F,G,H,Q,R,Pi);
 *             - BEGIN LOOP
 *                  - measure sensors, calculate y
 *                  - rc_kalman_update_lin(&kf, u, y);
 *                  - calculate new actuator control output u;
 *                  - save u for next loop;
 *             - END LOOP
 *             - rc_kalman_free(&kf);
 *             - return;
 *
 *             For the non-linear EKF case, the user should initialize the
 *             filter with rc_kalman_alloc_ekf() which does NOT take in the
 *             state transition matrices F,G,H. Instead it's up to the user to
 *             calculate the new predicted state x_pre and predicted sensor
 *             measurement h for their own non-linear system model each step.
 *             Those user-calculated predictions are then given to the
 *             non-linear rc_kalman_update_ekf() function.
 *
 *
 *             Basic loop structure for non-linear EKF case:
 *
 *             - rc_kalman_t kf = rc_kalman_empty();
 *             - rc_kalman_alloc_ekf(&kf,Q,R,Pi);
 *             - BEGIN LOOP
 *                  - measure sensors, calculate y
 *                  - calculate new Jacobian F given x_est from previous loop;
 *                  - calculate new predicted x_pre using x_est from previous loop;
 *                  - calulate new predicted sensor readings h from x_pre above;
 *                  - calculate new Jacobian H given x_pre;
 *                  - rc_kalman_update_ekf(&kf, F, x_pre, H, y, h);
 *                  - calculate new actuator control output u;
 *                  - save u for next loop;
 *             - END LOOP
 *             - rc_kalman_free(&kf);
 *             - return;
 *
 * @date       April 2018
 * @author     Eric Nauli Sihite & James Strawson
 *
 * @addtogroup Kalman
 * @ingroup    Math
 * @{
 */


#ifndef RC_KALMAN_H
#define RC_KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rc/math/vector.h>
#include <rc/math/matrix.h>


/*
 * @brief      Struct to contain full state of kalman filter
 */
typedef struct rc_kalman_t {
	/** @name Kalman Filter matrices */
	///@{
	rc_matrix_t P;		///< Predicted state error covariance
	rc_matrix_t Q;		///< Process noise covariance
	rc_matrix_t R;		///< Measurement noise covariance
	rc_matrix_t F;		///< undriven state-transition model
	rc_matrix_t G;		///< control input model
	rc_matrix_t H;		///< observation-model
	///@}

	/** @name State estimates */
	///@{
	rc_vector_t x_est;	///< Estimated state x[k|k]   = x[k|k-1],y[k])
	rc_vector_t x_pre;	///< Predicted state x[k|k-1] = f(x[k-1],u[k])
	///@}

	/** @name other */
	///@{
	rc_matrix_t Pi;		///< Initial P matrix
	int initialized;	///< set to 1 once initialized with rc_kalman_alloc
	uint64_t step;		///< counts times rc_kalman_measurement_update has been called
	///@}
} rc_kalman_t;

/**
 * @brief      Critical function for initializing rc_kalman_t structs
 *
 *             This is a very important function. If your rc_kalman_t struct is
 *             not a global variable, then its initial contents cannot be
 *             guaranteed to be anything in particular. Therefore it could
 *             contain problematic contents which could interfere with functions
 *             in this library. Therefore, you should always initialize your
 *             filters with rc_kalman_empty() before using with any other
 *             function in this library such as rc_kalman_alloc. This serves the
 *             same purpose as rc_matrix_empty, rc_vector_empty,
 *             rc_filter_empty, and rc_ringbuf_empty.
 *
 * @return     Empty zero-filled rc_kalman_t struct
 */
rc_kalman_t rc_kalman_empty();

/**
 * @brief      Allocates memory for a Kalman filter of given dimensions
 *
 * @param      kf    pointer to struct to be allocated
 * @param[in]  Q     Process noise covariance, can be updated later
 * @param[in]  R     Measurement noise covariance, can be updated later
 * @param[in]  Pi    Initial P matrix
 *
 * @return     0 on success, -1 on failure
 */
int rc_kalman_alloc_lin(rc_kalman_t* kf, rc_matrix_t F, rc_matrix_t G, rc_matrix_t H, rc_matrix_t Q, rc_matrix_t R, rc_matrix_t Pi);

/**
 * @brief      Allocates memory for a Kalman filter of given dimensions
 *
 * @param      kf    pointer to struct to be allocated
 * @param[in]  Nx    state vector length
 * @param[in]  Ny    measurement vector length
 * @param[in]  Nu    input vector length
 * @param[in]  Q     Process noise covariance, can be updated later
 * @param[in]  R     Measurement noise covariance, can be updated later
 * @param[in]  Pi    Initial P matrix
 *
 * @return     0 on success, -1 on failure
 */
int rc_kalman_alloc_ekf(rc_kalman_t* kf, rc_matrix_t Q, rc_matrix_t R, rc_matrix_t Pi);


/**
 * @brief      Frees the memory allocated by a kalman filter's matrices and
 *             vectors. Also resets all values to 0 like rc_kalman_empty().
 *
 * @param      kf    pointer to user's rc_kalman_t struct
 *
 * @return     0 on success or -1 on failure.
 */
int rc_kalman_free(rc_kalman_t* kf);


/**
 * @brief      reset state and dynamics of the filter to 0
 *
 *             Q and R are constant, and therefore not reset. P is set to
 *             identity*P_init.
 *
 * @param      kf    pointer to struct to be reset
 *
 * @return     0 on success, -1 on failure
 */
int rc_kalman_reset(rc_kalman_t* kf);


/**
 * @brief     Kalman Filter state prediction step based on physical model.
 *
 *            Uses the state estimate and control input from the previous timestep to produce an
 *            estimate of the state at the current timestep. This step pdates P and the estimated state x. Assume that you have calculated f(x[k|k],u[k]) and
 *            F(x[k|k],u[k]) before calling this function.
 *
 *            Kalman state prediction method
 *              - x_pre[k|k-1] = F*x[k-1|k-1] +  G*u[k-1]
 *              - P[k|k-1] = F*P[k-1|k-1]*F^T + Q
  *           - Kalman measurement Update:
                - h[k] = H * x_pre[k]
 *              - S = H*P*H^T + R
 *              - L = P*(H^T)*(S^-1)
 *              - x_est[k|k] = x[k|k-1] + L*(y[k]-h[k])
 *              - P[k|k] = (I - L*H)*P[k|k-1]
 *
 * @param      kf    pointer to struct to be updated
 * @param      u     control input
 *
 * @return     0 on success, -1 on failure
 */
int rc_kalman_update_lin(rc_kalman_t* kf, rc_vector_t u, rc_vector_t y);


/**
 * @brief      Kalman Filter measurement update step.
 *
 *             Updates L, P, & x. Assume that you have calculated h(x[k|k-1])
 *             and H(x[k|k-1]) beforehand.
 *
 *             Shorthand notations:
 *              - H = H(x[k|k-1])
 *              - P = P[k|k-1]
 *
 *             Kalman measurement Update:
  *             - P[k|k-1] = F*P[k-1|k-1]*F^T + Q
 *              - S = H*P*H^T + R
 *              - L = P*(H^T)*(S^-1)
 *              - x[k|k] = x[k|k-1] + L*y
 *              - P[k|k] = (I - L*H)*P
 *
 *             Also updates the step counter in the rc_kalman_t struct
 *
 * @param      kf    pointer to struct to be updated
 * @param      y     new sensor data vector
 * @param      h     Ideal estimate of y, usually h=H*x, but it is left to the
 *                   user to supply as an argument as real systems are usually
 *                   nonlinear and require more careful calculation of this.
 *
 * @return     0 on success, -1 on failure
 */
int rc_kalman_update_ekf(rc_kalman_t* kf, rc_matrix_t F, rc_vector_t x_pre, rc_matrix_t H, rc_vector_t y, rc_vector_t h);


#ifdef __cplusplus
}
#endif

#endif // RC_KALMAN_H

/** @}  end group math*/