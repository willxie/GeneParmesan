// g++ -Wall -Werror -Wextra kalman.cpp -o kalman

#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <math.h>

using namespace std;
using namespace Eigen;

const int DIM_X = 6;
const int DIM_U = 4;
const int DIM_Z = 2;
const double dt = 1.0/30;

// This implementation follows closely with Probilitistic Robotics book's
template<int DIM_X, int DIM_U, int DIM_Z>
class KalmanFilter {
  public:
    // Model params
    Matrix<double, DIM_X, DIM_X> A;
    Matrix<double, DIM_X, DIM_U> B;
    Matrix<double, DIM_Z, DIM_X> C;

    Matrix<double, DIM_X, DIM_X> pred_err;
    Matrix<double, DIM_Z, DIM_Z> sensor_err;

    // x_err = sigma
    Matrix<double, DIM_X, 1> x;
    Matrix<double, DIM_X, 1> x_pred;
    Matrix<double, DIM_X, DIM_X> x_err;
    Matrix<double, DIM_X, DIM_X> x_err_pred;

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

    Matrix<double, DIM_X, DIM_X> pred_err;
    Matrix<double, DIM_Z, DIM_Z> sensor_err;

    // x_err = sigma
    Matrix<double, DIM_X, 1> x;
    Matrix<double, DIM_X, 1> x_pred;
    Matrix<double, DIM_X, DIM_X> x_err;
    Matrix<double, DIM_X, DIM_X> x_err_pred;

    Matrix<double, DIM_X, 1> (*transitionFunction)(Matrix<double, DIM_X, 1>, Matrix<double, DIM_U, 1>);
    Matrix<double, DIM_Z, 1> (*measurementFunction)(Matrix<double, DIM_X, 1>);

    ExtendedKalmanFilter () {}

    // ExtendedKalmanFilter (Matrix<double, DIM_X, DIM_X> A,
    // 		      Matrix<double, DIM_X, DIM_U>& B,
    //               Matrix<double, DIM_Z, DIM_X>& C,
    //               Matrix<double, DIM_X, DIM_X>& pred_err,
	// 			  Matrix<double, DIM_Z, DIM_Z>& sensor_err,
	// 			  Matrix<double, DIM_X, 1>& x_0,
	// 			  Matrix<double, DIM_X, DIM_X>& x_err):
    //     A(A), B(B), C(C),
	// 	pred_err(pred_err), sensor_err(sensor_err),
	// 	x(x_0), x_err(x_err) {}

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
        	Matrix<double, DIM_X, 1> output_bar_temp = (*function)(input_temp, input_u);
        	// Fill in the ith row.
        	// The ith row takes the partial derivative of ith element of x
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
        calculateJacobianX(transitionFunction,  x, u, G);


        // State estimation
        x_pred = (*transitionFunction)(x, u);

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

    void updateWithoutAnything() {
        // Use Zero control
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


Matrix<double, DIM_X, 1> TransitionFunction(Matrix<double, DIM_X, 1> x, Matrix<double, DIM_U, 1> u) {
    u = u;
    Matrix<double, DIM_X, DIM_X> A;
    A << 1,    0,   dt,    0, pow(dt,2)/2,           0,
        0,    1,    0,   dt,           0, pow(dt,2)/2,
        0,    0,    1,    0,          dt,           0,
        0,    0,    0,    1,           0,          dt,
        0,    0,    0,    0,           1,           0,
        0,    0,    0,    0,           0,           1;
    return (A * x);
}

Matrix<double, DIM_Z, 1> MeasurementFunction(Matrix<double, DIM_X, 1> x) {
    Matrix<double, DIM_Z, DIM_X> C;
    C << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;
    return (C * x);
}

int main(int argc, char *argv[]) {
    argc = argc;
    argv = argv;

    Matrix<double, DIM_X, DIM_X> G = Matrix<double, DIM_X, DIM_X>::Zero();
    Matrix<double, DIM_Z, DIM_X> H = Matrix<double, DIM_Z, DIM_X>::Zero();
    Matrix<double, DIM_X, 1> x =  Matrix<double, DIM_X, 1>::Zero();
    Matrix<double, DIM_U, 1> u =  Matrix<double, DIM_U, 1>::Zero();

    cout << "x:" << endl;
    cout << x << endl;

    ExtendedKalmanFilter<DIM_X, DIM_U, DIM_Z> filter;
    cout << "." << endl;
    filter.calculateJacobianX(&TransitionFunction,  x, u, G);
    cout << "." << endl;
    filter.calculateJacobianZ(&MeasurementFunction, x,  H);
    cout << "." << endl;

    cout << "G:" << endl;
    cout << G << endl;
    cout << "H:" << endl;
    cout << H << endl;
}
// int main(int argc, char *argv[]) {
//     argc = argc;
//     argv = argv;

//     const int DIM = 4;

//     Matrix<double, DIM, DIM> A;
//     // TODO: B should be DIM x M where M is the dimensionality of the Ccntrol vector u
//     Matrix<double, DIM, DIM> B;
//     Matrix<double, DIM, DIM> C;

//     Matrix<double, DIM, DIM> pred_err;
//     Matrix<double, DIM, DIM> sensor_err;

//     Matrix<double, DIM, 1> x_0;
//     Matrix<double, DIM, DIM> x_err;

//     const double DELTA_T = 0.1;

//     // Dynamical model
//     A << 1, DELTA_T, 0,       0,
// 	     0,       1, 0,       0,
// 		 0,       0, 1, DELTA_T,
// 		 0,       0, 0,       1;

//     // This matrix only makes sense when combined with the u vector
//     B << 0, 0, 0, 0,
// 	     0, 0, 0, 0,
// 		 0, 0, 1, 0,
// 		 0, 0, 0, 1;

//     // No transformation from state to sensor land
//     C = Matrix<double, DIM, DIM>::Identity(DIM, DIM);

//     // No prediction error since it's deterministic
//     pred_err = Matrix<double, DIM, DIM>::Zero(DIM, DIM);

//     // Some sensor error
//     sensor_err = Matrix<double, DIM, DIM>::Identity(DIM, DIM) * 0.2;

//     // Initial guess. Totally wrong (?)
//     x_0 << 0,
//     	   100 * cos(M_PI/4),
// 		   500,
// 		   100 * sin(M_PI/4);

//     // Don't know any better
//     x_err = Matrix<double, DIM, DIM>::Identity(DIM, DIM);

//     // Initialize the filter
//     KalmanFilter<DIM, DIM, DIM> filter (A, B, C, pred_err, sensor_err, x_0, x_err);

//     const double GRAVITY = 9.81;
//     Matrix<double, 4, 1> u;
//     u << 0,
//     	 0,
// 		 -0.5 * GRAVITY * DELTA_T * DELTA_T,
// 		 -GRAVITY * DELTA_T;

//     int NOISE_LEVEL = 30;
//     int muzzle_vel = 100;
//     int angle = 45;
//     int ITERATIONS = 144;

//     // Cannon parameters
//     double x, y, x_new, y_new;
//     x = y = 0;

//     double x_vel, y_vel, x_vel_new, y_vel_new;
//     x_vel = muzzle_vel * cos(angle * M_PI/180);
//     y_vel = muzzle_vel * sin(angle * M_PI/180);

//     // Gaussian number generator
// 	std::default_random_engine generator;

//     double true_xs[ITERATIONS];
//     double true_ys[ITERATIONS];

//     double noisy_xs[ITERATIONS];
//     double noisy_ys[ITERATIONS];

//     double kalman_xs[ITERATIONS];
//     double kalman_ys[ITERATIONS];

//     Matrix<double, DIM, 1> z;
//     for (int i = 0; i < ITERATIONS; i++) {
//     	// True location of ball
//     	true_xs[i] = x;
//     	true_ys[i] = y;

//     	// Kalman estimate of ball's position
//     	Matrix<double, DIM, 1> kalman_state = filter.getState();
//     	kalman_xs[i] = kalman_state(0,0);
//     	kalman_ys[i] = kalman_state(2,0);

//     	// Generate noisy estimate
// 		std::normal_distribution<double> x_dist(x, NOISE_LEVEL);
// 		std::normal_distribution<double> y_dist(y, NOISE_LEVEL);
// 		noisy_xs[i] = x_dist(generator);
// 		noisy_ys[i] = y_dist(generator);

// 		// Update ball's true position
//     	x_new = x + x_vel*DELTA_T;
//     	y_new = y + y_vel*DELTA_T - 0.5*GRAVITY*DELTA_T*DELTA_T;

//     	x_vel_new = x_vel;
//     	y_vel_new = y_vel - GRAVITY*DELTA_T;

//     	x = x_new;
//     	y = y_new;
//     	x_vel = x_vel_new;
//     	y_vel = y_vel_new;

//     	// Give the new measurement and control input to the filter and let it update its belief
//     	z << noisy_xs[i],
//     		 x_vel_new,
// 			 noisy_ys[i],
// 			 y_vel_new;
//     	filter.update(u, z);
//     }

// 	cout << "xs = [";
// 	for (auto x : true_xs)
// 		cout << x << ", ";
// 	cout << "]" << endl;

// 	cout << "ys = [";
// 	for (auto y : true_ys)
// 		cout << y << ", ";
// 	cout << "]" << endl;
// 	cout << endl;

// 	cout << "noisy_xs = [";
// 	for (auto noisy_x : noisy_xs)
// 		cout << noisy_x << ", ";
// 	cout << "]" << endl;

// 	cout << "noisy_ys = [";
// 	for (auto noisy_y : noisy_ys)
// 		cout << noisy_y << ", ";
// 	cout << "]" << endl;
// 	cout << endl;

// 	cout << "kalman_xs = [";
// 	for (auto kalman_x : kalman_xs)
// 		cout << kalman_x << ", ";
// 	cout << "]" << endl;

// 	cout << "kalman_ys = [";
// 	for (auto kalman_y : kalman_ys)
// 		cout << kalman_y << ", ";
// 	cout << "]" << endl;

//     return 0;
// }
