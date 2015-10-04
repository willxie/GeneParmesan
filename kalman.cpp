// g++ -Wall -Werror -Wextra kalman.cpp -o kalman

#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// This implementation follows closely with Probilitistic Robotics book's
template<int DIM>
class KalmanFilter {
  public:
    // Model params
    Matrix<double, DIM, DIM> A;
    Matrix<double, DIM, DIM> B;
    Matrix<double, 1, DIM> C;
    Matrix<double, DIM, DIM> error_x;
    Matrix<double, DIM, 1> error_z;

    //  x = state, sigma = covariance matrix, u = control,, z = observation
    Matrix<double, DIM, 1> x_prev;
    Matrix<double, DIM, 1> x_pred;
    Matrix<double, DIM, 1> x;
    Matrix<double, DIM, DIM> sigma_prev;
    Matrix<double, DIM, DIM> sigma_pred;
    Matrix<double, DIM, DIM> sigma;
    double K;

    KalmanFilter (Matrix<double, DIM, DIM>& A, Matrix<double, DIM, DIM>& B,
                  Matrix<double, 1, DIM>& C,
                  Matrix<double, DIM, DIM>& error_x, Matrix<double, DIM, 1>& error_z,
                  Matrix<double, DIM, 1>& x_0):
        A(A), B(B), C(C), error_x(error_x), error_z(error_z), x_prev(x_0),
            sigma_prev(error_x) {}

    void update(Matrix<double, DIM, 1> u, Matrix<double, DIM, 1> z) {
        // State estimation
        x = (A * x_prev) + (B * u);
        sigma_pred = A * sigma_prev * A.transpose() + error_x;
        // Measurement update
        K = sigma_pred * C.transpose() * (C * sigma_pred * C.transpose() + error_z).inverse();
        x = x_pred + K * (z - C * x_pred);
        sigma = (Matrix<double, DIM, DIM>::Identity(DIM, DIM) - K * C) * sigma_pred;
        // Store values
        x_prev = x;
        sigma_prev = sigma;
    }

    Matrix<double, DIM, 1> getState() {
        return x;
    }

    Matrix<double, DIM, DIM> getCovariance() {
        return sigma;
    }

};

int main(int argc, char *argv[]) {
    argc = argc;
    argv = argv;

    const int DIM = 2;
    Matrix<double, DIM, DIM> A;
    Matrix<double, DIM, DIM> B;
    Matrix<double, 1, DIM> C;
    Matrix<double, DIM, DIM> error_x;
    Matrix<double, DIM, 1> error_z;
    Matrix<double, DIM, 1> x_0;

    // Construct model
    // e.g. x_0 = 0, 0;

    KalmanFilter<DIM> filter (A, B, C, error_x, error_z, x_0);

    return 0;
}
