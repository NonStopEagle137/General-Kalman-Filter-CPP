#define _GLIBCXX_USE_CXX11_ABI 0 // Can be removed.
#include <iostream>
#include <cstdio>
#include <vector>
#include <cmath>
#include "C:\DEV\vcpkg\packages\eigen3_x86-windows\include\eigen3\Eigen\Dense" // Path to your installation of Eigen.

using namespace std;
using namespace Eigen;



vector<VectorXd> measurements;

void filter(VectorXd &x, MatrixXd &P);

class KalmanFilter {
    public:
    VectorXd x; // object state
    MatrixXd P; // covariance matrix
    VectorXd u; // external motion
    MatrixXd H; // State transition Matrix
    MatrixXd F; // Measurement Matrix
    MatrixXd R; // Mesurement Covariance Matrix
    MatrixXd I; // Identity Matrix
    MatrixXd Q; // Process Covariance Matrix
    int measurement_count = 0;
    int observations = 0;
    KalmanFilter(int observations, int measurement_count){
        int ob = observations;
        this->x = VectorXd(ob);
        this->P = MatrixXd(ob,ob);
        this->u = VectorXd(ob);
        this->F = MatrixXd(ob,ob);
        this->H = MatrixXd(1,ob);
        this->R = MatrixXd(1,1);
        this->I = MatrixXd::Identity(ob,ob);
        this->Q = MatrixXd(ob,ob);
        this->observations = observations;
        this->measurement_count = measurement_count;

    }
    template<class dtype__> void vector_initialization(dtype__ &arr, double value = -1) {
        int r = 0;
        int c = 0;
        r = arr.rows();
        c = arr.cols();
        dtype__ temp;
        temp = dtype__::Identity(r,c);
        arr = temp; // Initializing to Identity matrix/vector.
        for (unsigned int i = 0; i < r; i++) {
            for (unsigned int j = 0; j < c; j++) {
                if (value == -1) {
                    arr(i,j) = pow(pow(pow((i+j), 1.05), 0.975), 1.02);
                }
                else {
                    arr(i,j) = value;
                }  
            }
        }
    }
    void generate_measurements() {
        VectorXd single_measurement(1);
        for (unsigned int i = 0; i < measurement_count ; i++) {
            single_measurement << i;
            measurements.push_back(single_measurement);
        }
        
    }

    int initialize_states_and_start() {

        int ob = observations;
        
        vector_initialization(P); // Violently variable covariance :)
        vector_initialization(u , 5); // Initializing u to 5(s).
        vector_initialization(F); // Variable Values for F
        vector_initialization(H);
        vector_initialization(R);
        vector_initialization(Q);

        generate_measurements();

        filter();

        getchar();

        return 0;

    }
    void filter() {
        for (unsigned int n = 0; n < measurements.size(); n++) {
            VectorXd z = measurements[n];
            VectorXd y = z - H * x;
            MatrixXd Ht = H.transpose();
            MatrixXd S = H * P * Ht + R;
            MatrixXd Si = S.inverse();
            MatrixXd K = P* Ht * Si;

            // Calculating the new state.

            x = x + (K * y);
            P = (I - K* H) * P;

            // Prediction Step.

            x = F * x + u;
            MatrixXd Ft = F.transpose();
            P = F * P * Ft + Q;

            std::cout<< " x = " << std::endl << x << std::endl;
            std::cout<< " p = " << std::endl << P << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    int observations = 5;
    int measurement_count = 40;
    KalmanFilter* filter_ = new KalmanFilter(observations,
                                            measurement_count);
    filter_->initialize_states_and_start();
}
