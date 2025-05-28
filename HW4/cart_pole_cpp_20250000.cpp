#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <fstream>

using namespace Eigen;
using namespace std;

const double m_1 = 1.0;
const double m_2 = 8.0;
const double I_1 = 0.024167;
const double I_2 = 1.01;
const double length = 0.6;
const double g = 9.81;

const double dt = 0.001;
const double T = 30.0;
const int steps = static_cast<int>(T / dt);

double F = 0;

////////////////// TODO //////////////////////
/// Add variables if necessary



////////////////// TODO END //////////////////

Matrix<double, 6, 6> massMatrix() {

    ////////////////// TODO //////////////////////
    /// Calculate the mass matrix (Part2. 2-1)

    Matrix<double, 6, 6> M = Matrix<double, 6, 6>::Zero();  // should be modified

    ////////////////// TODO END //////////////////

    return M;
}

void updateJandJ_dot(const VectorXd& q, const VectorXd& q_dot, MatrixXd& J, MatrixXd& J_dot) {

    ////////////////// TODO //////////////////////
    /// Calculate the Jacobian and Jacobian_dot matrices (Part2. 2-1)



    ////////////////// TODO END //////////////////
}

VectorXd updateFext() {

    ////////////////// TODO //////////////////////
    /// Calculate the external force vector (Part2. 2-1)

    VectorXd Fext = VectorXd::Zero(6); // should be modified

    ////////////////// TODO END //////////////////

    return Fext;
}

void pid_controller()
{
    ////////////////// TODO //////////////////////
    /// Calculate the force to keep the pole vertical (Part3. 3-1)

    F = 0;  // should be modified

    ////////////////// TODO END //////////////////
}

VectorXd computeQddot(const VectorXd& q, const VectorXd& q_dot, const Matrix<double, 6, 6>& M) {
    MatrixXd J, J_dot;
    updateJandJ_dot(q, q_dot, J, J_dot);
    pid_controller();
    VectorXd Fext = updateFext();

    VectorXd b_top = Fext;
    VectorXd b_bottom = -J_dot * q_dot;
    VectorXd b(b_top.size() + b_bottom.size());
    b << b_top, b_bottom;

    MatrixXd A(M.rows() + J.rows(), M.cols() + J.rows());
    A << M, J.transpose(),
        J, MatrixXd::Zero(J.rows(), J.rows());

    VectorXd x = A.colPivHouseholderQr().solve(b);
    return x.head(6);
}

int main() {
    // initial conditions
    VectorXd q(6), q_dot(6);
    q(0) = 0;
    q(1) = 0;
    q(2) = 0;
    q(3) = 0;
    q(4) = 0;
    q(5) = 0.523599;
    q_dot.setZero();

    auto M = massMatrix();

    ofstream log_file("results_20250000.csv");
    log_file << "Time,Position,theta,Force\n";

    vector<VectorXd> log;
    VectorXd temp_log(4);
    double count = 0;
    temp_log << dt * count, q(0), q(5), F;
    log.push_back(temp_log);
    count++;

    for (int i = 0; i < steps; ++i) {
        ////////////////// TODO //////////////////////
        /// Implement the semi-implicit Euler method (Part2. 2-2)



        ////////////////// TODO END //////////////////

        temp_log << dt * count, q(0), q(5), F;
        log.push_back(temp_log);
        count++;
    }


    for (size_t i = 0; i < log.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            log_file << log[i](j) << (j < 3 ? ',' : '\n');
        }
    }
    log_file.close();
    cout << "Simulation complete. results_20250000.csv »ý¼ºµÊ." << endl;
    return 0;
}