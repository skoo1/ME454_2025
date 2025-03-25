// Matrix library source code for KAIST ME454 Dynamics System Programming course
// 2024 March, Gunwoo Park

#include "../include/mymat.hpp"

// MatBase CLASS STARTS HERE

// Constructor of the MatBase class
// Initializes [rows] x [cols] zero matrix
// Throws error if [rows] or [cols] are bigger than the maximum value
MatBase::MatBase(int rows, int cols)
{
    if (rows > max_size) throw std::runtime_error("Error : rows exceeds the limit\n");
    if (cols > max_size) throw std::runtime_error("Error : cols exceeds the limit\n");

    m_rows = rows;
    m_cols = cols;

    // TODO : Implement zero matrix initialization
}

// Destructor of the MatBase class
// Implement it if you need 
MatBase::~MatBase()
{
}

// Set an element in ([row], [col]) to [elem] of the matrix
void MatBase::set_elem(int row, int col, double elem)
{
    if (row >= m_rows) throw std::runtime_error("Error : row out of the range\n");
    if (col >= m_cols) throw std::runtime_error("Error : row out of the range\n");
    // TODO : Set [elem] to the container
}

// Returns an element in ([row], [col]) of the matrix
double MatBase::get_elem (int row, int col) const
{
    // TODO : Get [elem] from the container
    return 0.0;
}

// Matrix visualization
// Already implemented
void MatBase::display ()
{
    std::cout << "Matrix : " << std::endl;
    for (int i_row = 0; i_row < m_rows; i_row++)
    {
        for (int i_col = 0; i_col < m_cols; i_col++) std::cout << get_elem(i_row, i_col) << "\t";
        std::cout << "\n";
    }
    std::cout << std::endl;
} 
// Assignment operator (=)
// Assigns the matrix in [mat] to [this] matrix
// Syntax : [this] = [mat]
// Already implemented
void MatBase::operator= (const MatBase &mat) 
{
    if (mat.m_rows != m_rows) throw std::runtime_error("Error : rows are not the same\n");
    if (mat.m_cols != m_cols) throw std::runtime_error("Error : cols are not the same\n");

    for (int i_row = 0; i_row < m_rows; i_row++) for (int i_col = 0; i_col < m_cols; i_col++)
        set_elem(i_row, i_col, mat.get_elem(i_row, i_col));
}

// Addition operator (+)
// Returns the [sum] of [this] and [mat]
// Syntax : [sum] = [this] + [mat]
MatBase MatBase::operator+ (const MatBase &mat) 
{
    if (mat.m_rows != m_rows) throw std::runtime_error("Error : rows are not the same\n");
    if (mat.m_cols != m_cols) throw std::runtime_error("Error : cols are not the same\n");

    MatBase sum(m_rows, m_cols);
    // TODO : Implement addition

    return sum;
}

// Subtraction operator (-)
// Returns the difference [diff] of [this] and [mat]
// Syntax : [diff] = [this] - [mat]
MatBase MatBase::operator- (const MatBase &mat) 
{
    if (mat.m_rows != m_rows) throw std::runtime_error("Error : rows are not the same\n");
    if (mat.m_cols != m_cols) throw std::runtime_error("Error : cols are not the same\n");

    MatBase diff(m_rows, m_cols);
    // TODO : Implement subtraction
    
    return diff;
}

// Matrix Multiplication operator (*) -> polymorphism
// Returns the product [prod] of [this] and [mat]
// Syntax : [prod] = [this] * [mat]
MatBase MatBase::operator* (const MatBase &mat) 
{
    if (mat.m_rows != m_cols) throw std::runtime_error("Error : the first rows and second cols are not the same\n");

    MatBase prod(m_rows, mat.m_cols);
    // TODO : Implement matrix multiplication
    
    return prod;
}

// Scalar Multiplication operator (*) -> polymorphism
// Returns the product of [this] and [val]
// Syntax : [prod] = [this] * [val]
MatBase MatBase::operator* (double val) 
{
    MatBase prod(m_rows, m_cols);
    // TODO : Implement scalar multiplication

    return prod;
}


// Mat33 CLASS STARTS HERE

// Constructor of the Mat33 class
// Initializes 3x3 zero matrix
Mat33::Mat33()
{
    m_rows = 3;
    m_cols = 3;
    // TODO : Implement zero matrix initialization
}

// Constructor of the Mat33 class
// Initializes 3x3 diagonal matrix with [eii]
Mat33::Mat33(double e00, double e11, double e22)
{
    m_rows = 3;
    m_cols = 3;
    // TODO : Implement diagonal matrix initialization
}

// Constructor of the Mat33 class
// Initializes 3x3 matrix with [eij]
Mat33::Mat33(  double e00, double e01, double e02,
                double e10, double e11, double e12,
                double e20, double e21, double e22  )
{
    m_rows = 3;
    m_cols = 3;
    // Implement matrix initialization (optional, this will not be evaluated.)
}

// Transpose matrix calculation
// Returns the transpose of [this]
Mat33 Mat33::transpose()
{
    Mat33 trans;
    // TODO : Implement transpose

    return trans;
}

// Determinant calculation (Optional)
// Returns the determinant of [this]
double Mat33::det()
{
    double val = 0.0;
    // Implement determinant calculation (optional, this will not be evaluated.)

    return val;
}

// Inverse matrix calculation
// Returns the inverse of [this]
Mat33 Mat33::inverse()
{
    Mat33 inv;
    // TODO : Implement inverse matrix calculation
    
    return inv;
}


// Vec3 CLASS STARTS HERE

// Constructor of the Vec3 class
// Initializes 3d zero vector (same as 3x1 matrix)
Vec3::Vec3()
{
    m_rows = 3;
    m_cols = 1;
    // TODO : Implement zero vector initialization
}

// Constructor of the Vec3 class
// Initializes 3d vector with [e0, e1, e2]
Vec3::Vec3(double e0, double e1, double e2)
{
    m_rows = 3;
    m_cols = 1;
    // TODO : Implement vector initialization
}

// Dot product calculation
// Returns the dot product of [this] and [vec]
double Vec3::dot(const Vec3 &vec)
{
    double prod = 0.0;
    // TODO : Implement dot product

    return prod;
}

// Cross product calculation
// Returns the cross product of [this] and [vec]
Vec3 Vec3::cross(const Vec3 &vec)
{
    Vec3 prod;
    // TODO : Implement cross product

    return prod;
}

// Skew symmetric matrix calculation
// Returns the skew symmetric matrix of [this]
Mat33 Vec3::skew()
{
    Mat33 mat;
    // TODO : Implement skew symmetric matrix calculation

    return mat;
}

// Squared norm calculation
// Returns the squared norm of [this]
double Vec3::squarednorm()
{
    double val = 0.0;
    // Implement squared norm calculation (optional, this will not be evaluated.)

    return val;
}

// Vector visualization
// Already implemented
void Vec3::display()
{
    std::cout << "Vector : " << std::endl;
    for (int i_row = 0; i_row < m_rows; i_row++)
    {
        for (int i_col = 0; i_col < m_cols; i_col++) std::cout << get_elem(i_row, i_col) << "\t";
        std::cout << "\n";
    }
    std::cout << std::endl;
} 