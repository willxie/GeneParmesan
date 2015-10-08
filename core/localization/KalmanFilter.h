#ifndef TRUNK_CORE_LOCALIZATION_KALMANFILTER_H_
#define TRUNK_CORE_LOCALIZATION_KALMANFILTER_H_

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// Kalman filter parameters
const int DIM_X = 6;
const int DIM_U = 4;
const int DIM_Z = 2;
const double dt = 1.0/30;   // This is 1/(sample rate). Assuming 30 hz


// This implementation follows closely with Probilitistic Robotics book's
template<int DIM_X, int DIM_U, int DIM_Z>
class KalmanFilter {
  public:
    // Model params
    Matrix<double, DIM_X, DIM_X> A;
    Matrix<double, DIM_X, DIM_U> B;
    Matrix<double, DIM_Z, DIM_X> C;

    // State transition error and sensation error (pred_error = R, sensor_err = Q)
    Matrix<double, DIM_X, DIM_X> pred_err;
    Matrix<double, DIM_Z, DIM_Z> sensor_err;

    // State and covariance terms (x_err = sigma)
    Matrix<double, DIM_X, 1> x;
    Matrix<double, DIM_X, 1> x_pred;
    Matrix<double, DIM_X, DIM_X> x_err;
    Matrix<double, DIM_X, DIM_X> x_err_pred;

    // Constructors
    KalmanFilter () {}

    KalmanFilter (Matrix<double, DIM_X, DIM_X> A,
    		      Matrix<double, DIM_X, DIM_U>& B,
                  Matrix<double, DIM_Z, DIM_X>& C,
                  Matrix<double, DIM_X, DIM_X>& pred_err,
				  Matrix<double, DIM_Z, DIM_Z>& sensor_err,
				  Matrix<double, DIM_X, 1>& x_0,
				  Matrix<double, DIM_X, DIM_X>& x_err):
        A(A), B(B), C(C),
		pred_err(pred_err), sensor_err(sensor_err),
		x(x_0), x_err(x_err) {}

    // Update using both control and sensor values
    void update(Matrix<double, DIM_U, 1> u, Matrix<double, DIM_Z, 1> z) {
        // State estimation
        x_pred = (A * x) + (B * u);
        x_err_pred = A * x_err * A.transpose() + pred_err;

        // Measurement update
        Matrix<double, DIM_X, DIM_Z> K = x_err_pred * C.transpose() *
            (C * x_err_pred * C.transpose() + sensor_err).inverse();
        x_pred = x_pred + K * (z - C * x_pred);

        x_err_pred = (Matrix<double, DIM_X, DIM_X>::Identity(DIM_X, DIM_X) - K * C) * x_err_pred;

        // Store values
        x = x_pred;
        x_err = x_err_pred;
    }

    // Update using solely state transition prediction with no control and sensation
    void updateWithoutAnything() {
        // State estimation
        x_pred = (A * x);
        x_err_pred = A * x_err * A.transpose() + pred_err;

        // Measurement update
        Matrix<double, DIM_X, DIM_Z> K = x_err_pred * C.transpose() *
            (C * x_err_pred * C.transpose() + sensor_err).inverse();

        // Store values
        x = x_pred;
        x_err = x_err_pred;
    }

    Matrix<double, DIM_X, 1> getState() {
        return x;
    }

    Matrix<double, DIM_X, DIM_X> getCovariance() {
        return x_err;
    }

};


template<int DIM_X, int DIM_U, int DIM_Z>
class ExtendedKalmanFilter {
  public:
    // Model params
    Matrix<double, DIM_X, DIM_X> G;
    Matrix<double, DIM_Z, DIM_X> H;

    // State transition error and sensation error (pred_error = R, sensor_err = Q)
    Matrix<double, DIM_X, DIM_X> pred_err;
    Matrix<double, DIM_Z, DIM_Z> sensor_err;

    // State and covariance terms (x_err = sigma)
    Matrix<double, DIM_X, 1> x;
    Matrix<double, DIM_X, 1> x_pred;
    Matrix<double, DIM_X, DIM_X> x_err;
    Matrix<double, DIM_X, DIM_X> x_err_pred;

    // Arbitrary functions that model transition and sensation (g(u, x), h(x_pred))
    Matrix<double, DIM_X, 1> (*transitionFunction)(Matrix<double, DIM_X, 1>, Matrix<double, DIM_U, 1>);
    Matrix<double, DIM_Z, 1> (*measurementFunction)(Matrix<double, DIM_X, 1>);

    // Constructors
    ExtendedKalmanFilter () {}

    // Calculate the Jacobian matrix based on the input function
    void calculateJacobianX(Matrix<double, DIM_X, 1> (*function)(Matrix<double, DIM_X, 1>, Matrix<double, DIM_U, 1>),
                            Matrix<double, DIM_X, 1> input_x,
                            Matrix<double, DIM_U, 1> input_u,
                            Matrix<double, DIM_X, DIM_X>& G) {
    	double delta = 0.01; // Increment used for numerical derivative
    	Matrix<double, DIM_X, 1> output_bar = (*function)(input_x, input_u);
    	for (int i = 0; i < DIM_X; ++i) {
    		// Add delta to x one dimension at a time
    		Matrix<double, DIM_X, 1> input_temp = input_x;
    		input_temp(i, 0) = input_temp(i, 0) + delta;
            // Feed modified input to get output
        	Matrix<double, DIM_X, 1> output_bar_temp = (*function)(input_temp, input_u);
        	// Fill in the ith column with the derivative.
        	// The ith column takes the partial derivative of ith element of x
        	G.col(i) = (output_bar_temp - output_bar) / delta;

    	}
    }

    void calculateJacobianZ(Matrix<double, DIM_Z, 1> (*function)(Matrix<double, DIM_X, 1>),
                            Matrix<double, DIM_X, 1> input,
                            Matrix<double, DIM_Z, DIM_X>& H) {
    	double delta = 0.01; // Increment used for numerical derivative
    	Matrix<double, DIM_Z, 1> output_bar = (*function)(input);
    	for (int i = 0; i < DIM_X; ++i) {
    		// Add delta to x one dimension at a time
    		Matrix<double, DIM_X, 1> input_temp = input;
    		input_temp(i, 0) = input_temp(i, 0) + delta;
        	Matrix<double, DIM_Z, 1> output_bar_temp = (*function)(input_temp);
        	// Fill in the ith row.
        	// The ith row takes the partial derivative of ith element of x
        	H.col(i) = (output_bar_temp - output_bar) / delta;
    	}
    }

    void update(Matrix<double, DIM_U, 1> u,
                Matrix<double, DIM_Z, 1> z) {
        // Calcuate the Jacobian matrix G using the current state and control
        calculateJacobianX(transitionFunction, x, u, G);

        // State estimation
        x_pred = (*transitionFunction)(x, u);

        // Calcuate the Jacobian matrix H using the predicted state calculated above
        calculateJacobianZ(measurementFunction, x_pred,  H);
        x_err_pred = G * x_err * G.transpose() + pred_err;

        // Measurement update
        Matrix<double, DIM_X, DIM_Z> K = x_err_pred * H.transpose() *
            (H * x_err_pred * H.transpose() + sensor_err).inverse();
        x_pred = x_pred + K * (z - (*measurementFunction)(x_pred));

        x_err_pred = (Matrix<double, DIM_X, DIM_X>::Identity(DIM_X, DIM_X) - K * H) * x_err_pred;

        // Store values
        x = x_pred;
        x_err = x_err_pred;
    }

    // Update using solely state transition prediction with no control and sensation
    void updateWithoutAnything() {
        // Use Zero control since we can't explicitly take out the u term
        Matrix<double, DIM_U, 1> u = Matrix<double, DIM_U, 1>::Zero();
        // State estimation
        x_pred = (*transitionFunction)(x, u);
        x_err_pred = G * x_err * G.transpose() + pred_err;

        // Measurement update
        Matrix<double, DIM_X, DIM_Z> K = x_err_pred * H.transpose() *
            (H * x_err_pred * H.transpose() + sensor_err).inverse();

        // Store values
        x = x_pred;
        x_err = x_err_pred;
    }

    Matrix<double, DIM_X, 1> getState() {
        return x;
    }

    Matrix<double, DIM_X, DIM_X> getCovariance() {
        return x_err;
    }

};

#endif /* TRUNK_CORE_LOCALIZATION_KALMANFILTER_H_ */
