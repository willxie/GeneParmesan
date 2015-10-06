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

    Matrix<double, DIM_X, 1> getState() {
        return x;
    }

    Matrix<double, DIM_X, DIM_X> getCovariance() {
        return x_err;
    }

};

#endif /* TRUNK_CORE_LOCALIZATION_KALMANFILTER_H_ */
