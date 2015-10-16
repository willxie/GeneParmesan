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

template<int DIM_X, int DIM_U, int DIM_Z>
class ParticleFilter {
  public:
    // Model params
    int num_particles;
    std::vector<Matrix<double, DIM_X, 1> > particles, candidate_particles;

    ParticleFilter () {}

    /* Populate particles with M random particles */
    void init() {
    	for (int i = 0; i < num_particles; i++) {
    		//particles[i] =
    	}
    }

    void update(Matrix<double, DIM_U, 1> u, Matrix<double, DIM_Z, 1> z) {
        for (int i = 0; i < m; ++i) {
            Matrix<double, DIM_X, 1> x;
            x = (*g)(u, particles[i]);
            // TODO: calculate weight for distance and angle
            candidate_particles[i] = x;
        }

        // Resample
        for (int i = 0; i < num_particles; ++i) {
            Matrix<double, DIM_X, 1> x;
            // TODO: resmple
            particles[i] = x;
        }
    }
};

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
