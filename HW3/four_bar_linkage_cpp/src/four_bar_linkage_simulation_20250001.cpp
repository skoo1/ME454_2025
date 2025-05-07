#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

double L1_, L2_, L3_, L0_;
double m1_, m2_, m3_;
double ixx1_;
double ixx2_;
double ixx3_;
double y1_init_, y2_init_, y3_init_, z1_init_,z2_init_,z3_init_;
double th1_init_, th2_init_, th3_init_;
double th1dot_init_, th2dot_init_, th3dot_init_;

Eigen::MatrixXd M_(9,9), M_inv_(9,9);

double g = 9.81;

void initialize(){

    ////////////////// TODO //////////////////////
    // Edit the value of the properties

    L0_ = 0.0;

    m1_ = 0.0;
    L1_ = 0.0;
    ixx1_ = 0.0;
    th1_init_ = 0.0;
    y1_init_ = 0.0;
    z1_init_ = 0.0;

    m2_ = 1.0;
    L2_ = 0.0;
    ixx2_ = 0.0;
    th2_init_ = 0.0;
    y2_init_ = 0.0;
    z2_init_ = 0.0;

    m3_ = 0.0;
    L3_ = 0.0;
    ixx3_ = 0.0;
    th3_init_ = 0.0;
    y3_init_ = 0.0;
    z3_init_ = 0.0;

    // ToDo: set the mass matrix and the inversion of the mass matrix
    M_.setZero();
    M_inv_.setZero();

    ////////////////// TODO END //////////////////////

}

Eigen::MatrixXd calculate_jacobian(Eigen::VectorXd &q){
    Eigen::MatrixXd J;
    ////////////////// TODO //////////////////////
    /// Calculate the Jacobian using the input state vector q



    ////////////////// TODO END //////////////////////
    return J;
}

Eigen::MatrixXd calculate_jacobian_dot(Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::MatrixXd Jdot;

    ////////////////// TODO //////////////////////
    /// Calculate the time derivative of Jacobian using the input state vectors q and qdot



    ////////////////// TODO END //////////////////////
    return Jdot;
}

Eigen::VectorXd calculate_Fext(Eigen::VectorXd q){
    Eigen::VectorXd Fext;
    ////////////////// TODO //////////////////////
    /// Calculate the external force using the input state vector q



    ////////////////// TODO END //////////////////////
    return Fext;
}

Eigen::VectorXd calculate_lambda(Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd Fext, Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::VectorXd lambda;
    ////////////////// TODO //////////////////////
    /// Calculate lambda which connects the constraint force and the jacobian



    ////////////////// TODO END //////////////////////
    return lambda;
}

Eigen::VectorXd calculate_Fc(Eigen::MatrixXd J, Eigen::VectorXd lambda){
    Eigen::VectorXd Fc;
    ////////////////// TODO //////////////////////
    /// Calculate the constraint force using the Jacobian and lambda



    ////////////////// TODO END //////////////////////
    return Fc;
}

Eigen::VectorXd calculate_constraint_error(Eigen::VectorXd q){
    Eigen::VectorXd ConstraintErr;
    ////////////////// TODO //////////////////////
    /// Calculate the constraint errors under given state vector q



    ////////////////// TODO END //////////////////
    return ConstraintErr;
}

int main()
{
    std::ofstream q_results("q_results_cpp.csv");
    std::ofstream qdot_results("qdot_results_cpp.csv");
    std::ofstream constraint_error_results("constraint_error_cpp.csv");

    double h = 0.001; // simulation timestep, 0.001 seconds (1 ms)
    int n_sim = 50000; // number of simulation steps (1000 s)
    int rec_steps = 10; // record the result every n steps (every 0.1 s)
    initialize();

    Eigen::VectorXd q(9);
    Eigen::VectorXd qdot(9);

    std::cout << "Simulation start" << std::endl;
    
    //////////////// TODO ////////////////
    // TODO: you can set yout own variavble.

    double t = 0;

    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        
        
        // TODO: implement semi-implicit Euler method to simulate the four bar linkage motion

        if (i_sim % rec_steps == 0)
        {
            q_results << t << ",";
            qdot_results << t << ",";
            // TODO: record q_results_cpp.csv contains the value of the angle q1, q2, and q3 in radian (q1,q2,q3)
            
            q_results <<"\n";

            // TODO: record qdot_results_cpp.csv contains the value of the angular velocity qdot1, qdot2, and qdot3 in radian (qdot1,qdot2,qdot3)

            qdot_results <<"\n";

            // TODO: record constraint_error_cpp.csv contains the constraint error at this time step (error1,error2,...)

            constraint_error_results << "\n";
        }
        //////////////// TODO end ////////////////
        t = t + h;
    }
    std::cout << "Simulation end" << std::endl;

    // close the file
    q_results.close();
    qdot_results.close();
    constraint_error_results.close();

    return 0;
}


