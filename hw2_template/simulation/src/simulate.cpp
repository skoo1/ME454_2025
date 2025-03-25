#include <iostream>
#include <fstream>
#include <string>
#include <mymat.hpp>

int main()
{
    // create and open file to record simulation results
    std::ofstream file_results("results.csv");

    double h = 0.0001; // simulation timestep, 0.0001 seconds (0.1 ms)
    int n_sim = 6000000; // number of simulation step
    int rec_steps = 1000; // record the result every n steps (every 0.1 s)
    
    Mat33 I_lcs(1.8, 2.0, 2.2); // inertia of the body (local coordinate system)
    std::cout << "I_lcs" << std::endl;
    I_lcs.display();

    Mat33 R_g2l(1.0, 1.0, 1.0); // rotation matrix from global to local
    std::cout << "R_g2l" << std::endl;
    R_g2l.display();

    Vec3 w_lcs(0.0, 1.0, 0.01); // angular velocity from global to local (local coordinate system)
    std::cout << "w_lcs" << std::endl;
    w_lcs.display();

    Vec3 wd_lcs;
    Mat33 eye(1.0, 1.0, 1.0);

    std::cout << "Simulation start" << std::endl;
    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        // TODO: implement semi-implicit Euler method to simulate the tennis racket theorem

        // record the simulation results to the file
        if (i_sim % rec_steps == 0)
        {
            file_results << i_sim * h << ", "; // record time
            // record the orientation of the local frame
            for (int i_row = 0; i_row < 3; i_row++) for (int i_col = 0; i_col < 3; i_col++) file_results << R_g2l.get_elem(i_row, i_col) << ", "; 
            file_results << "\n";
        }
    }
    std::cout << "Simulation end" << std::endl;

    // close the file
    file_results.close();
    
    return 0;
}