#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <fstream>

using namespace Eigen;
using namespace std;

const double m1 = 0.20, m2 = 0.30;
const double l1 = 0.10, l2 = 0.15;
const double I1 = m1 * (2*l1) * (2*l1) / 12.0;
const double I2 = m2 * (2*l2) * (2*l2) / 12.0;
const double g = 9.81;

const double dt = 0.001;
const double T = 1.55;
const int steps = static_cast<int>(T / dt);

Matrix<double, 6, 6> massMatrix() {
    Matrix<double, 6, 6> M = Matrix<double, 6, 6>::Zero();
    M.diagonal() << m1, m1, I1, m2, m2, I2;
    return M;
}

void updateJ(const VectorXd &q, const VectorXd &q_dot, MatrixXd &J, MatrixXd &J_dot) {
    double x1 = q(0), y1 = q(1), th1 = q(2);
    double x2 = q(3), y2 = q(4), th2 = q(5);
    double w1 = q_dot(2), w2 = q_dot(5);
    double s1 = sin(th1), c1 = cos(th1);
    double s2 = sin(th2), c2 = cos(th2);

    J.resize(5, 6);
    J << 1, 0,  l1*s1,   0, 0,  0,
         0, 1, -l1*c1,   0, 0,  0,
         0, 0,  2*l1*s1, 1, 0,  l2*s2,
         0, 0, -2*l1*c1, 0, 1, -l2*c2,
         0, 0,  2*l1*c1, 0, 0,  2*l2*c2;

    J_dot.resize(5, 6);
    J_dot << 0, 0,  l1*w1*c1,   0, 0,  0,
             0, 0,  l1*w1*s1,   0, 0,  0,
             0, 0,  2*l1*w1*c1, 0, 0,  l2*w2*c2,
             0, 0,  2*l1*w1*s1, 0, 0,  l2*w2*s2,
             0, 0, -2*l1*w1*s1, 0, 0, -2*l2*w2*s2;
}

VectorXd updateFext() {
    VectorXd F(6);
    F << 0, -m1 * g, 0, 0, -m2 * g, 0;
    return F;
}

VectorXd computeQddot(const VectorXd &q, const VectorXd &q_dot, const Matrix<double, 6, 6> &M) {
    MatrixXd J, J_dot;
    updateJ(q, q_dot, J, J_dot);
    VectorXd F_ext = updateFext();

    VectorXd b_top = F_ext;
    VectorXd b_bottom = -J_dot * q_dot;
    VectorXd b(b_top.size() + b_bottom.size());
    b << b_top, b_bottom;

    MatrixXd A(M.rows() + J.rows(), M.cols() + J.rows());
    A << M, J.transpose(),
         J, MatrixXd::Zero(J.rows(), J.rows());

    VectorXd x = A.colPivHouseholderQr().solve(b);
    // colPivHouseholderQr() performs QR decomposition with column pivoting, 
    // which improves numerical stability, especially for nearly singular 
    // or ill-conditioned matrices.
    // solve(b) uses the decomposed form of AA to directly compute 
    // the solution without explicitly computing the inverse of AA. This 
    // is both faster and more memory efficient.

    return x.head(6); // q_ddot
}

int main() {
    double theta1 = 1.0;
    double theta2 = asin(std::clamp(-2 * l1 * sin(theta1) / (2 * l2), -1.0, 1.0));

    double x1 = l1 * cos(theta1);
    double y1 = l1 * sin(theta1);
    double x2 = 2 * l1 * cos(theta1) + l2 * cos(theta2);
    double y2 = 2 * l1 * sin(theta1) + l2 * sin(theta2);

    VectorXd q(6), q_dot(6);
    q << x1, y1, theta1, x2, y2, theta2;
    q_dot.setZero();

    auto M = massMatrix();

    vector<VectorXd> q_hist, q_dot_hist;
    q_hist.push_back(q);
    q_dot_hist.push_back(q_dot);

    for (int i = 0; i < steps; ++i) {
        VectorXd q_ddot = computeQddot(q, q_dot, M);
        q_dot += q_ddot * dt;
        q += q_dot * dt;
        q_hist.push_back(q);
        q_dot_hist.push_back(q_dot);
    }

    ofstream fout("q_trajectory.csv");
    for (const auto &q_vec : q_hist) {
        for (int i = 0; i < q_vec.size(); ++i) {
            fout << q_vec(i);
            if (i < q_vec.size() - 1) fout << ",";
        }
        fout << "\n";
    }
    fout.close();

    cout << "Simulation complete. Results written to q_trajectory.csv" << endl;
    return 0;
}