// g++ -Wall -Werror -Wextra kalman.cpp -o kalman

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <random>
#include <math.h>
#include <map>

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

    ParticleFilter () {}

    /* Populate particles with M random particles */
    void init() {
    	for (int i = 0; i < num_particles; i++) {
    		//particles[i] =
    	}
    }

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

        // Resample
        for (int i = 0; i < num_particles; ++i) {
            Matrix<double, DIM_X, 1> x;
            // TODO: resmple
            particles[i] = x;
        }
    }

int main(int argc, char *argv[]) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::discrete_distribution<> d({20,30,200,10});
	std::map<int, int> m;

	for(int n = 0; n < 10000; n++) {
		m[d(gen)]++;
	}

	for(auto p : m) {
		std::cout << p.first << " generated " << p.second << " times\n";
	}
}
