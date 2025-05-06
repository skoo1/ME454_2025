#include <iostream>
#include <cmath>

#define PI 3.141592

void explicit_euler_integration(
    double theta0, 
    double omega0, 
    double dt, 
    double &theta1, 
    double &omega1, 
    double *gxyg0, 
    double *vgxyg0, 
    double *gxyg1, 
    double *vgxyg1, 
    double )
{
        // update the pose of the racket
        theta1 = theta0 + omega0 * dt;
        omega1 = omega0;

        gxyg1[0] = gxyg0[0] + vgxyg0[0] * dt;
        gxyg1[1] = gxyg0[1] + vgxyg0[1] * dt;
        vgxyg1[0] = vgxyg0[0];
        vgxyg1[1] = vgxyg0[1] - 9.8 * dt;
}

int main()
{
    double pt[][2] = {{25,  200}, {55,  250}, {100, 280}, {160, 300}, 
        {210, 300}, {275, 290}, {312, 270}, {370, 240}, {420, 210}, {495, 180},
        {560, 175}, {635, 175}, {710, 175}, {710, 135}, {635, 135}, {560, 135}, 
        {495, 130}, {420, 100}, {370, 70}, {312, 40}, {275, 20}, {210, 10}, 
        {160, 10}, {100, 30}, {55,  60}, {25,  110}}; 

    double gxyoff[2]; // in meter, coordinate of COM
    int npt = 26; // number of mass points on the racket
    double mp = 0.01; // in kg, mass of a point

    double sum_px_mp = 0.0, sum_py_mp = 0.0;
    for (int i = 0; i<npt; i++) {
        sum_px_mp += pt[i][0] * 0.001 * mp;
        sum_py_mp += pt[i][1] * 0.001 * mp;	
    }

    gxyoff[0] = sum_px_mp / (npt * mp);
    gxyoff[1] = sum_py_mp / (npt * mp);

    std::cout << "gx = " << gxyoff[0] << std::endl;
    std::cout << "gy = " << gxyoff[1] << std::endl; 

    // Initial conditions
    double theta0 = -120.0*PI/180.0;
    double gxyg0[2] = {1.0, 1.0};
    double omega0 = -180.0*PI/180.0;
    double vgxyg0[2] = {5.0, 5.0};

    // Initial pose calculation
    double oxyg[2];
    oxyg[0] = gxyg0[0] - (cos(theta0)*gxyoff[0] - sin(theta0)*gxyoff[1]);
    oxyg[1] = gxyg0[1] - (sin(theta0)*gxyoff[0] + cos(theta0)*gxyoff[1]);

    std::cout << "=== points at 0 ===" << std::endl;
    double ptg[26][2];
    for (int i=0; i<npt; i++) {
        ptg[i][0] = oxyg[0] + (cos(theta0)*pt[i][0]*0.001 - sin(theta0)*pt[i][1]*0.001);
        ptg[i][1] = oxyg[1] + (sin(theta0)*pt[i][0]*0.001 + cos(theta0)*pt[i][1]*0.001);
        std::cout << ptg[i][0] << ", " << ptg[i][1] << std::endl;
    }

    double dt = 0.1;
    for (int j=0; j<10; j++) {
        double theta1, omega1, gxyg1[2], vgxyg1[2];

        explicit_euler_integration(theta0, omega0, dt, 
            theta1, omega1, gxyg0, vgxyg0, gxyg1, vgxyg1)

        // draw the points on the racket
        oxyg[0] = gxyg1[0] - (cos(theta1)*gxyoff[0] - sin(theta1)*gxyoff[1]);
        oxyg[1] = gxyg1[1] - (sin(theta1)*gxyoff[0] + cos(theta1)*gxyoff[1]);

        std::cout << "=== one step forward ===" << std::endl;
        for (int i=0; i<npt; i++) {
            ptg[i][0] = oxyg[0] + (cos(theta1)*pt[i][0]*0.001 - sin(theta1)*pt[i][1]*0.001);
            ptg[i][1] = oxyg[1] + (sin(theta1)*pt[i][0]*0.001 + cos(theta1)*pt[i][1]*0.001);
            std::cout << ptg[i][0] << ", " << ptg[i][1] << std::endl;
        }

        // prepare for the next step
        theta0 = theta1;
        omega0 = omega1;
        gxyg0[0] = gxyg1[0];
        gxyg0[1] = gxyg1[1];
        vgxyg0[0] = vgxyg1[0];
        vgxyg0[1] = vgxyg1[1];
    }

    return 0;
}


