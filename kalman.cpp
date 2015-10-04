// g++ -Wall -Werror -Wextra kalman.cpp -o kalman

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <random>

using namespace std;
using namespace Eigen;

// This implementation follows closely with Probilitistic Robotics book's
template<int DIM>
class KalmanFilter {
  public:
    // Model params
    Matrix<double, DIM, DIM> A;
    Matrix<double, DIM, DIM> B;
    Matrix<double, DIM, DIM> C;

    Matrix<double, DIM, DIM> pred_err;
    Matrix<double, DIM, 1> sensor_err;

    //  x = state, sigma = covariance matrix, u = control,, z = observation
    Matrix<double, DIM, 1> x;
    Matrix<double, DIM, 1> x_pred;
    Matrix<double, DIM, DIM> x_err;
    Matrix<double, DIM, DIM> x_err_pred;
    double K;

    KalmanFilter (Matrix<double, DIM, DIM>& A,
    		      Matrix<double, DIM, DIM>& B,
                  Matrix<double, DIM, DIM>& C,
                  Matrix<double, DIM, DIM>& pred_err,
				  Matrix<double, DIM, DIM>& sensor_err,
				  Matrix<double, DIM, 1>& x_0,
				  Matrix<double, DIM, DIM>& x_err):
        A(A), B(B), C(C),
		pred_err(pred_err), sensor_err(sensor_err),
		x_err(x_err), x(x_0) {}

    void update(Matrix<double, DIM, 1> u, Matrix<double, DIM, 1> z) {
        // State estimation
        x_pred = (A * x) + (B * u);
        x_err_pred = A * x_err * A.transpose() + pred_err;

        // Measurement update
        K = x_err_pred * C.transpose() * (C * x_err_pred * C.transpose() + sensor_err).inverse();
        x_pred = x_pred + K * (z - C * x_pred);
        x_err_pred = (Matrix<double, DIM, DIM>::Identity(DIM, DIM) - K * C) * x_err_pred;

        // Store values
        x = x_pred;
        x_err = x_err_pred;
    }

    Matrix<double, DIM, 1> getState() {
        return x;
    }

    Matrix<double, DIM, DIM> getCovariance() {
        return x_err;
    }

};

int main(int argc, char *argv[]) {
    const int DIM = 1;

    Matrix<double, DIM, DIM> A;
    // TODO: B should be DIM x M where M is the dimensionality of the Ccntrol vector u
    Matrix<double, DIM, DIM> B;
    Matrix<double, DIM, DIM> C;

    Matrix<double, DIM, DIM> error_a;
    Matrix<double, DIM, DIM> error_z;

    Matrix<double, DIM, 1> x_0;
    Matrix<double, DIM, DIM> error_x;

    // Dynamical model
    A(0,0) = 1; // State is not changing
    B(0,0) = 0; // No control input
    C(0,0) = 1; // z is already in state coordinates

    // Sources of error
    error_a(0,0) = 0.00001;  // Prediction error
    error_z(0,0) = 0.1;      // Measurement error

    // Initial estimates
    x_0(0,0) = 3;      // Initial estimate of the state
    error_x(0,0) = 1;  // Initial state error

    // Initialize the filter
    KalmanFilter<DIM> filter (A, B, C, error_a, error_z, x_0, error_x);

    const int NUM_SAMPLES = 60;
    const double TRUE_VOLTAGE = 1.25;

    // Gaussian number generator
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(TRUE_VOLTAGE, 0.25);

    double true_voltages[NUM_SAMPLES];
    double measures[NUM_SAMPLES];
    double kalmans[NUM_SAMPLES];
    for (int i = 0; i < NUM_SAMPLES; ++i) {
    	// Generate a reading from the voltage meter
        double measured = distribution(generator);

        measures[i] = measured;
        true_voltages[i] = TRUE_VOLTAGE;
        Matrix<double, DIM, 1> x = filter.getState();
        kalmans[i] = x(0,0);

        // Send it into the Kalman Filter
        Matrix<double, DIM, DIM> z;
        z(0,0) = measured;
        filter.update(Matrix<double, DIM, DIM>::Zero(DIM, DIM), z);
    }

    cout << "measured = [";
    for (auto measure : measures)
    	cout << measure << ", ";
    cout << "]" << endl;

    cout << "true_voltages = [";
    for (auto true_voltage : true_voltages)
    	cout << true_voltage << ", ";
    cout << "]" << endl;

    cout << "kalmans = [";
    for (auto kalman : kalmans)
    	cout << kalman << ", ";
    cout << "]" << endl;

    return 0;
}
