/* compile this with g++ -Wall -lm -L. -lselforg  -o example matrixexample.cpp or use the Makefile*/

#include <iostream>
#include <math.h>

#include <selforg/matrix.h>

using namespace matrix;
using namespace std;

int main(){
  ////////// matrix construction
  //  all matrices are initialised with 0 elements
  Matrix m0;    // 0-matrix (size is 0x0)
  m0.set(5,1);  // change matrix to be a 5x1 matrix (column vector)
  Matrix m1(2,3); // 2x3-matrix
  double data[9] = {1., 0., 1.,  0.1, 0.4, 0.5,  -2., 0.1, 2.};
  m1.set(3,3,data); // change matrix to be 3x3 matrix with initialised elements
  Matrix m2(3,3,data); // 3x3 matrix with initialised elements

  ////////// Accessing and printing
  cout << m2.val(0,0) << endl; // Element with index 0,0
  m2.val(2,1) = 3.4;           // assign value to element at 2,1
  cout << m2.val(2,1) << endl; // Element with index 2,1

  cout << m2;                  // show matrix
  cout << m2.row(0);           // show first row which is row-vector
  cout << m2.column(2);        // show third column which is column-vector

  ////////// Operations
  Matrix m3 = m2 + m2 - m2 * m2;         // addition subtraction and multiplication
  cout << m3;
  Matrix m4 = (m3^T) * (m3^-1) * (m3.multMT()); // transpose and invert and fast version of matrix*matrix^T
                                               // (parentheses needed because ^ bind less than *)
  Matrix m5 = m4*0.3;                    // multiplication with scalar (just right side)
  Matrix m6 = m4 & (m3.column(0)); // multiply m4 rowwise,
                                   // each row with the appropriate element of first column of m3

  Matrix m7 = m6.map(sin);         // apply sin function to all matrix elements
  cout << m7;

  return 0;
}
