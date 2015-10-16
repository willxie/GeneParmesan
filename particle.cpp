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

// Noise variables
const double noise_mean = 0.0;
const double noise_stddev_xy = 10; // TODO tweak this
const double noise_stddev_theta = 0.2; // TODO tweak this




Matrix<double, DIM_X, 1> TransitionFunction(Matrix<double, DIM_X, 1> x, Matrix<double, DIM_U, 1> u) {
    Matrix<double, DIM_X, DIM_X> A = Matrix<double, DIM_X, DIM_X>::Identity();
    Matrix<double, DIM_U, DIM_U> B = Matrix<double, DIM_U, DIM_U>::Identity();
    // x = x + x * dt
    B = B * dt;

    // Noise
    std::random_device rd;
    std::mt19937 gen (rd());
    std::normal_distribution<double> nd_x (noise_mean, noise_stddev_xy);
    std::normal_distribution<double> nd_y (noise_mean, noise_stddev_xy);
    std::normal_distribution<double> nd_theta (noise_mean, noise_stddev_theta);

    Matrix<double, DIM_X, 1> noise;

    noise <<     nd_x(gen) <<     nd_y(gen) <<     nd_theta(gen);

    return (A * x) + (B * u) + noise;
}



template<int DIM_X, int DIM_U, int DIM_Z>
class ParticleFilter {
  public:
    // Model params
    int num_particles;
    std::vector<Matrix<double, DIM_X, 1> > particles, candidate_particles, weights;

    Matrix<double, DIM_X, 1> (*transitionFunction)(Matrix<double, DIM_X, 1>, Matrix<double, DIM_U, 1>);
    // Matrix<double, DIM_Z, 1> (*measurementFunction)(Matrix<double, DIM_X, 1>);

    ParticleFilter ()  {}

    // Loc is the location of the beacon (x, y)
    void update(Matrix<double, DIM_U, 1> u, Matrix<double, DIM_Z, 1> z,  Matrix<double, 2, 1> loc) {
        for (int i = 0; i < m; ++i) {
            Matrix<double, DIM_X, 1> x;
            // Motion with noise
            x = (*transitionFunction)(particles[i], u);

            // Weights
            double distance_particle = sqrt(pow(x(0) - loc(0), 2) + pow(x(1) - loc(1), 2));
            double distance_measurement = z(0);
            double weight_distance = MAX_DIST - abs(distance_particle - distance_measurement);
            // TODO: dunno how to calculate bearing lol
            // double angle_particle = 0;
            double weight_angle    = MAX_ANGLE - 0;
            const weight_ratio  = 1; // TODO move this out
            double weight_combined = weight_ratio * weight_distance  + (1 - weight_ratio) * weight_angle;

            candidate_particles[i] = x;
            weights[i] = weight_combined;
        }
        for (int i = 0; i < m; ++i) {
            Matrix<double, DIM_X, 1> x;
            // TODO: resmple
            particles[i] = x;
        }
    }
    void resample() {

    }

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

int main(int argc, char *argv[]) {
    argc = argc;
    argv = argv;

    Matrix<double, DIM_X, DIM_X> G = Matrix<double, DIM_X, DIM_X>::Zero();
    Matrix<double, DIM_Z, DIM_X> H = Matrix<double, DIM_Z, DIM_X>::Zero();
    Matrix<double, DIM_X, 1> x =  Matrix<double, DIM_X, 1>::Zero();
    Matrix<double, DIM_U, 1> u =  Matrix<double, DIM_U, 1>::Zero();

    cout << "x:" << endl;
    cout << x << endl;

    ParticleFilter<DIM_X, DIM_U, DIM_Z> filter;
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
