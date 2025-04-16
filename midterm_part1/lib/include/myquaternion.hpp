// Quaternion library source code for KAIST ME454 Dynamics System Programming course
// 2024 May, Beomsoo Shin
// Modified by Gunwoo Park

#ifndef _MYQUATERNION_H_
#define _MYQUATERNION_H_
#include "mymat.hpp"
#include <cmath>
#include <limits> // For std::numeric_limits
const double _FLOAT_EPS = std::numeric_limits<double>::epsilon(); // Define a small epsilon value for floating-point comparisons
const double _EPS4 = _FLOAT_EPS * 4.0;

class Quaternion : public Vec4
{
    public:

        Quaternion(); // constructor
        Quaternion(double w, double x, double y, double z); // constructor with values, [w, x, y, z]
        Quaternion(const Vec4& vec) : Vec4(vec) {  };// cast to MatBase

        void set_w(double elem);
        void set_x(double elem);
        void set_y(double elem);
        void set_z(double elem);

        double get_w() const;
        double get_x() const;
        double get_y() const;
        double get_z() const;

        double norm(); // quaternion norm

        Quaternion normalize(); // quaternion normalization
        Quaternion conjugate(); // quaternion conjugate
        Quaternion inverse(); // quaternion inverse
        
        Quaternion operator^ (const Quaternion &quat); // Hamilton product q1^q2 = qh
        
        void display(); // print the quaternion

};

#endif // _MYQUATERNION_H_
