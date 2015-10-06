// g++ -Wall -Werror -Wextra kalman.cpp -o kalman

#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <math.h>

using namespace std;
using namespace Eigen;

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

    Matrix<double, DIM_X, 1> getState() {
        return x;
    }

    Matrix<double, DIM_X, DIM_X> getCovariance() {
        return x_err;
    }

};

int main(int argc, char *argv[]) {
    argc = argc;
    argv = argv;

    const int DIM = 4;

    Matrix<double, DIM, DIM> A;
    // TODO: B should be DIM x M where M is the dimensionality of the Ccntrol vector u
    Matrix<double, DIM, DIM> B;
    Matrix<double, DIM, DIM> C;

    Matrix<double, DIM, DIM> pred_err;
    Matrix<double, DIM, DIM> sensor_err;

    Matrix<double, DIM, 1> x_0;
    Matrix<double, DIM, DIM> x_err;

    const double DELTA_T = 0.1;

    // Dynamical model
    A << 1, DELTA_T, 0,       0,
	     0,       1, 0,       0,
		 0,       0, 1, DELTA_T,
		 0,       0, 0,       1;

    // This matrix only makes sense when combined with the u vector
    B << 0, 0, 0, 0,
	     0, 0, 0, 0,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

    // No transformation from state to sensor land
    C = Matrix<double, DIM, DIM>::Identity(DIM, DIM);

    // No prediction error since it's deterministic
    pred_err = Matrix<double, DIM, DIM>::Zero(DIM, DIM);

    // Some sensor error
    sensor_err = Matrix<double, DIM, DIM>::Identity(DIM, DIM) * 0.2;

    // Initial guess. Totally wrong (?)
    x_0 << 0,
    	   100 * cos(M_PI/4),
		   500,
		   100 * sin(M_PI/4);

    // Don't know any better
    x_err = Matrix<double, DIM, DIM>::Identity(DIM, DIM);

    // Initialize the filter
    KalmanFilter<DIM, DIM, DIM> filter (A, B, C, pred_err, sensor_err, x_0, x_err);

    const double GRAVITY = 9.81;
    Matrix<double, 4, 1> u;
    u << 0,
    	 0,
		 -0.5 * GRAVITY * DELTA_T * DELTA_T,
		 -GRAVITY * DELTA_T;

    int NOISE_LEVEL = 30;
    int muzzle_vel = 100;
    int angle = 45;
    int ITERATIONS = 144;

    // Cannon parameters
    double x, y, x_new, y_new;
    x = y = 0;

    double x_vel, y_vel, x_vel_new, y_vel_new;
    x_vel = muzzle_vel * cos(angle * M_PI/180);
    y_vel = muzzle_vel * sin(angle * M_PI/180);

    // Gaussian number generator
	std::default_random_engine generator;

    double true_xs[ITERATIONS];
    double true_ys[ITERATIONS];

    double noisy_xs[ITERATIONS];
    double noisy_ys[ITERATIONS];

    double kalman_xs[ITERATIONS];
    double kalman_ys[ITERATIONS];

    Matrix<double, DIM, 1> z;
    for (int i = 0; i < ITERATIONS; i++) {
    	// True location of ball
    	true_xs[i] = x;
    	true_ys[i] = y;

    	// Kalman estimate of ball's position
    	Matrix<double, DIM, 1> kalman_state = filter.getState();
    	kalman_xs[i] = kalman_state(0,0);
    	kalman_ys[i] = kalman_state(2,0);

    	// Generate noisy estimate
		std::normal_distribution<double> x_dist(x, NOISE_LEVEL);
		std::normal_distribution<double> y_dist(y, NOISE_LEVEL);
		noisy_xs[i] = x_dist(generator);
		noisy_ys[i] = y_dist(generator);

		// Update ball's true position
    	x_new = x + x_vel*DELTA_T;
    	y_new = y + y_vel*DELTA_T - 0.5*GRAVITY*DELTA_T*DELTA_T;

    	x_vel_new = x_vel;
    	y_vel_new = y_vel - GRAVITY*DELTA_T;

    	x = x_new;
    	y = y_new;
    	x_vel = x_vel_new;
    	y_vel = y_vel_new;

    	// Give the new measurement and control input to the filter and let it update its belief
    	z << noisy_xs[i],
    		 x_vel_new,
			 noisy_ys[i],
			 y_vel_new;
    	filter.update(u, z);
    }

	cout << "xs = [";
	for (auto x : true_xs)
		cout << x << ", ";
	cout << "]" << endl;

	cout << "ys = [";
	for (auto y : true_ys)
		cout << y << ", ";
	cout << "]" << endl;
	cout << endl;

	cout << "noisy_xs = [";
	for (auto noisy_x : noisy_xs)
		cout << noisy_x << ", ";
	cout << "]" << endl;

	cout << "noisy_ys = [";
	for (auto noisy_y : noisy_ys)
		cout << noisy_y << ", ";
	cout << "]" << endl;
	cout << endl;

	cout << "kalman_xs = [";
	for (auto kalman_x : kalman_xs)
		cout << kalman_x << ", ";
	cout << "]" << endl;

	cout << "kalman_ys = [";
	for (auto kalman_y : kalman_ys)
		cout << kalman_y << ", ";
	cout << "]" << endl;

    return 0;
}
