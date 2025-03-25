#include "../include/mymat.hpp"

int main()
{
    // Test any functions you want. This code will not be evaluated.

    double elems[9] = {3.0, 1.0, 4.0, 1.0, 5.0, 9.0, 2.0, 6.0, 5.0};
    
    Mat33 mat_a, mat_inv, mat_eye;
    for (int row = 0; row < 3; row++) for (int col = 0; col < 3; col++) mat_a.set_elem(row, col, elems[row * 3 + col]);

    mat_inv = mat_a.inverse();
    mat_eye = mat_a * mat_inv;

    std::cout << "This is A" << std::endl;
    mat_a.display();
    std::cout << "This is inv(A)" << std::endl;
    mat_inv.display();
    std::cout << "This is A * inv(A)" << std::endl;
    mat_eye.display();

    return 0;
}