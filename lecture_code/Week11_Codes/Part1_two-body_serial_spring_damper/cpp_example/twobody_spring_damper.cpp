#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <fstream>

using namespace Eigen;
using namespace std;

const double m1 = 0.20, m2 = 0.10;
const double l1 = 0.50, l2 = 0.25;
const double I1 = m1 * (2*l1) * (2*l1) / 12.0;
const double I2 = m2 * (2*l2) * (2*l2) / 12.0;
const double g  = 9.81;
const double K  = 10.0, C = 2.0;
const double L0 = 0.05;   // nominal length of the spring

const double dt = 0.001;
const double T  = 2.0;
const int steps = static_cast<int>(T / dt);

Matrix<double,6,6> massMatrix() {
    Matrix<double,6,6> M = Matrix<double,6,6>::Zero();
    M.diagonal() << m1, m1, I1, m2, m2, I2;
    return M;
}

void updateJ(const VectorXd &q, const VectorXd &q_dot, MatrixXd &J, MatrixXd &J_dot) {
    double x1 = q(0), y1 = q(1), th1 = q(2);
    double x2 = q(3), y2 = q(4), th2 = q(5);
    double vx1 = q_dot(0), vy1 = q_dot(1), w1 = q_dot(2);
    double vx2 = q_dot(3), vy2 = q_dot(4), w2 = q_dot(5);
    double c = cos(th1), s = sin(th1);
    double dx = x2 - x1, dy = y2 - y1;

    J.resize(4,6);
    J <<  1, 0, -l1*c, 0, 0, 0,
          0, 1, -l1*s, 0, 0, 0,
         -c, -s, -dx*s+dy*c, c, s, 0,
          0, 0, 1, 0, 0,-1;

    J_dot = MatrixXd::Zero(4,6);
    J_dot(0,2) =  l1 * s * w1;
    J_dot(1,2) = -l1 * c * w1;
    J_dot(2,0) =  s * w1;
    J_dot(2,1) = -c * w1;
    J_dot(2,2) = -vx2*s - x2*c*w1 + vx1*s + x1*c*w1 + vy2*c - y2*s*w1 - vy1*c + y1*s*w1;
    J_dot(2,3) = -s * w1;
    J_dot(2,4) =  c * w1;
}

// external force or applied force
VectorXd updateFext(const VectorXd &q, const VectorXd &q_dot) {
    double x1 = q(0), y1 = q(1), th1 = q(2);
    double x2 = q(3), y2 = q(4), th2 = q(5);
    double vx1 = q_dot(0), vy1 = q_dot(1), w1 = q_dot(2);
    double vx2 = q_dot(3), vy2 = q_dot(4), w2 = q_dot(5);
    double c = cos(th1), s = sin(th1);

    // force by spring and damper
    double L    = sqrt(x2*x2 + y2*y2) - 2*l1 - l2 - L0;
    double Ldot = (x2*vx2 + y2*vy2) / sqrt(x2*x2 + y2*y2);
    double Fsd  = K * L + C * Ldot;

    VectorXd Fext = VectorXd::Zero(6);
    Fext(0) =  Fsd * s;
    Fext(1) = -Fsd * c - m1 * g;
    Fext(3) = -Fsd * s;
    Fext(4) =  Fsd * c - m2 * g;
    return Fext;
}

// q̈ calculation: [M  Jᵀ; J 0] x = [Fext; -J̇ q̇]
VectorXd computeQddot(const VectorXd &q, const VectorXd &q_dot, const Matrix<double,6,6> &M) {
    MatrixXd J, J_dot;
    updateJ(q, q_dot, J, J_dot);
    VectorXd Fext = updateFext(q, q_dot);

    VectorXd b_top    = Fext;
    VectorXd b_bottom = -J_dot * q_dot;
    VectorXd b(b_top.size() + b_bottom.size());
    b << b_top, b_bottom;

    MatrixXd A(M.rows()+J.rows(), M.cols()+J.rows());
    A << M, J.transpose(),
         J, MatrixXd::Zero(J.rows(), J.rows());

    VectorXd x = A.colPivHouseholderQr().solve(b);
    return x.head(6);
}

int main() {
    // initial conditions
    double theta1 = M_PI/4, theta2 = M_PI/4;
    VectorXd q(6), q_dot(6);
    q(0) = l1 * sin(theta1);
    q(1) = -l1 * cos(theta1);
    q(2) = theta1;
    q(3) = (2*l1 + L0 + l2) * sin(theta2);
    q(4) = -(2*l1 + L0 + l2) * cos(theta2);
    q(5) = theta2;
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

    // save results in files
    ofstream fq("q_hist.csv"), fqd("q_dot_hist.csv");
    for(size_t i=0; i<q_hist.size(); ++i){
        for(int j=0; j<6; ++j){
            fq << q_hist[i](j)  << (j<5?',' : '\n');
            fqd<< q_dot_hist[i](j) << (j<5?',' : '\n');
        }
    }
    fq.close(); fqd.close();
    cout<<"Simulation complete. q_hist.csv & q_dot_hist.csv 생성됨."<<endl;
    return 0;
}