/*
 * KalmanFilter.h
 *
 *  Created on: Oct 5, 2015
 *      Author: ebanner
 */

#ifndef TRUNK_CORE_LOCALIZATION_KALMANFILTER_H_
#define TRUNK_CORE_LOCALIZATION_KALMANFILTER_H_

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
    Matrix<double, DIM, DIM> C;

    Matrix<double, DIM, DIM> pred_err;
    Matrix<double, DIM, DIM> sensor_err;

    //  x = state, sigma = covariance matrix, u = control, z = observation
    Matrix<double, DIM, 1> x;
    Matrix<double, DIM, 1> x_pred;
    Matrix<double, DIM, DIM> x_err;
    Matrix<double, DIM, DIM> x_err_pred;

    KalmanFilter () {}

    KalmanFilter (Matrix<double, DIM, DIM>& A,
    		      Matrix<double, DIM, DIM>& B,
                  Matrix<double, DIM, DIM>& C,
                  Matrix<double, DIM, DIM>& pred_err,
				  Matrix<double, DIM, DIM>& sensor_err,
				  Matrix<double, DIM, 1>& x_0,
				  Matrix<double, DIM, DIM>& x_err):
        A(A), B(B), C(C),
		pred_err(pred_err), sensor_err(sensor_err),
		x(x_0), x_err(x_err) {}

    void update(Matrix<double, DIM, 1> u, Matrix<double, DIM, 1> z) {
        // State estimation
        x_pred = (A * x) + (B * u);
        x_err_pred = A * x_err * A.transpose() + pred_err;

        // Measurement update
        Matrix<double, DIM, DIM> innovation_cov = C * x_err_pred * C.transpose() + sensor_err;
        Matrix<double, DIM, DIM> K = x_err_pred * C.transpose() * innovation_cov.inverse();
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

#endif /* TRUNK_CORE_LOCALIZATION_KALMANFILTER_H_ */
