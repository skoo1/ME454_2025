// Matrix library header for KAIST ME454 Dynamics System Programming course
// 2024 March, Gunwoo Park

#ifndef MYMAT_H_
#define MYMAT_H_
#include <iostream>
#include <vector>

const int max_size = 8;

// Base matrix class
class MatBase
{
    public: // accessible outside the class
        
        MatBase() {m_rows = 0; m_cols = 0;}; // for Mat33 and Vec3 inheritance
        MatBase(int rows, int cols); // constructor
        ~MatBase(); // destructor
        
        void set_elem (int row, int col, double elem); // set an element from the matrix
        double get_elem (int row, int col) const; // get an element from the matrix
        void display (); // matrix visualization (optional)

        void operator= (const MatBase &mat); // assignment
        MatBase operator+ (const MatBase &mat); // addition
        MatBase operator- (const MatBase &mat); // subtraction
        MatBase operator* (const MatBase &mat); // matrix multiplication
        MatBase operator* (double val); // scalar multiplication

    protected: // accessible only for class and subclass member functions
        
        int m_cols, m_rows;

        // TODO : add any kind of variable that can store the matrix elements

        // declare variables if you need
};

class Mat33;
class Vec3;

// 3x3 matrix class (subclass of MatBase)
// Inherits member functions and variables from MatBase
class Mat33 : public MatBase
{
    public:

        Mat33(); // constructor
        Mat33(double e00, double e11, double e22); // constructor with diagonal values
        Mat33(  double e00, double e01, double e02,
                double e10, double e11, double e12,
                double e20, double e21, double e22  ); // constructor with all values (optional)
        Mat33(const MatBase& base) : MatBase(base) { } // cast to MatBase

        Mat33 transpose(); // transpose
        double det(); // determinant (optional)
        Mat33 inverse(); // inverse
};

// 3d vector class (subclass of MatBase)
// Inherits member functions and variables from MatBase
class Vec3 : public MatBase
{
    public:
        Vec3(); // constructor
        Vec3(double e0, double e1, double e2); // constructor with values
        Vec3(const MatBase& base) : MatBase(base) {  } // cast to MatBase

        double dot(const Vec3 &vec); // dot product
        Vec3 cross(const Vec3 &vec); // cross product
        Mat33 skew(); // cross product of two vectors
        double squarednorm(); // squared norm (optional)

        void display(); // vector visualization (optional)
};

#endif