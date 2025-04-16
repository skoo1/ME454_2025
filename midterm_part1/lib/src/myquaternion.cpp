// Quaternion library source code for KAIST ME454 Dynamics System Programming course
// 2024 May, Beomsoo Shin
// Modified by Gunwoo Park

#include "../include/myquaternion.hpp"
#include <cmath> // For std::numeric_limits
#include <limits> // For std::numeric_limits

// constructor for empty argument -> identity quaternion
Quaternion::Quaternion() 
{
    set_elem(0, 0, 1.0);
    set_elem(1, 0, 0.0);
    set_elem(2, 0, 0.0);
    set_elem(3, 0, 0.0);
}

// constructor with [w, x, y, z] values
Quaternion::Quaternion(double w, double x, double y, double z) 
{
    set_elem(0, 0, w);
    set_elem(1, 0, x);
    set_elem(2, 0, y);
    set_elem(3, 0, z);
}

// set quaternion values
void Quaternion::set_w(double elem) { set_elem(0, 0, elem); } 
void Quaternion::set_x(double elem) { set_elem(1, 0, elem); } 
void Quaternion::set_y(double elem) { set_elem(2, 0, elem); } 
void Quaternion::set_z(double elem) { set_elem(3, 0, elem); } 

// get quaternion values
double Quaternion::get_w() const { return get_elem(0, 0); } 
double Quaternion::get_x() const { return get_elem(1, 0); } 
double Quaternion::get_y() const { return get_elem(2, 0); } 
double Quaternion::get_z() const { return get_elem(3, 0); } 

// quaternion norm
double Quaternion::norm() { return sqrt(squarednorm()); }

// quaternion normalization (to unit quaternion)
Quaternion Quaternion::normalize()
{
    Quaternion unit(get_w() / norm(), get_x() / norm(), get_y() / norm(), get_z() / norm());
    return unit;
}

// quaternion conjugate operation
Quaternion Quaternion::conjugate()
{
    Quaternion conj(get_w(), -get_x(), -get_y(), -get_z());
    return conj;
}

// quaternion Hamilton product
Quaternion Quaternion::operator^(const Quaternion &quat)
{
    double w = get_w() * quat.get_w() - get_x() * quat.get_x() - get_y() * quat.get_y() - get_z() * quat.get_z(); 
    double x = get_w() * quat.get_x() + get_x() * quat.get_w() + get_y() * quat.get_z() - get_z() * quat.get_y();
    double y = get_w() * quat.get_y() - get_x() * quat.get_z() + get_y() * quat.get_w() + get_z() * quat.get_x();
    double z = get_w() * quat.get_z() + get_x() * quat.get_y() - get_y() * quat.get_x() + get_z() * quat.get_w();

    Quaternion prod(w, x, y, z);
    return prod;
}

// quaternion inverse operation
Quaternion Quaternion::inverse()
{
    return Quaternion(conjugate() * (1.0 / squarednorm()));
}

// print quaternion
void Quaternion::display()
{
    std::cout << "Quaternion : \t";
    std::cout << get_elem(0, 0) << "\t";
    std::cout << get_elem(1, 0) << "\t";
    std::cout << get_elem(2, 0) << "\t";
    std::cout << get_elem(3, 0) << "\t";
    std::cout << std::endl;
}
