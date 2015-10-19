#include <iostream>
#include <math.h>


using namespace std;

// m = mean, s = stddev, x = target point
double calculateGaussianValue(double m, double s, double x) {
    return ( 1 / ( s * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x-m)/s, 2.0 ) );    
}

int main(int argc, char *argv[]) {
    double x = 0;
    double m = 0;
    double s = 10;
    
    for (int x = 0; x < 30; ++x) {
        double x1_gaussian = calculateGaussianValue(m, s, m);
        double x2_gaussian = calculateGaussianValue(m, s, x);
        double gaussian = (x1_gaussian - fabs(x1_gaussian - x2_gaussian)) / x1_gaussian;
        double linear = (100 - fabs(x - m)) / 100;
        
        cout << "gaussian: " << gaussian << endl;
        cout << "linear: " << linear << endl;
    }
}


