#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H


#include <vector>

#include "exceptions.h"
#include "vector.h"
#include "matrices.h"


namespace university_of_leipzig {
namespace robots {


template<typename T>
class CubicSpline {
 protected: 
 typedef std::vector< Matrix<T> > MatrixSet;

 protected:
  MatrixSet coefficient_matrix_set; // one matrix for each component
  Vector<T> parameter_vector;       // size of this is col_count of matrix + 1

 public:
  CubicSpline();
 

  void create(const Matrix<T> &r_matrix);

  Vector<T> get_point(T parameter);


  T get_distant_point_parameter(T parameter, T distance);
};


template<class T>
CubicSpline<T>::CubicSpline()
{
}

/**
 * .) each component is processed individually
 *
 * .) calculate the delta_t s:
 *      delta_t[0] = t[2] - t[0]
 *      delta_t[i] = t[i + 1] - t[i] | i >= 1 .. n
 *
 * .) calculate the ks:
 *      k[i] = (f[i] - f[i - 1]) / delta_t
 *
 *
 * result matrix: 4 x (n - 1)
 * 0: a = f
 * 1: b
 * 2: c
 * 3: d
 * 4: t
 *
 *
 * input : a matrix, first row must be the t parameters
 * output: is stored internally,
 *         for each component a matrix consisting of a, b, c and d is created
 *         the t parameter is stored seperately
 */

template<class T>
void CubicSpline<T>::create(const Matrix<T> &r_matrix)
{
  unsigned n = r_matrix.get_column_count();
  unsigned m = r_matrix.get_row_count();

  // create the matrix for the c coefficents
  TriDiagonalMatrix<T> mat_c(n - 1);

  coefficient_matrix_set.resize(m - 1);

  parameter_vector.resize(n);

  // copy the parameters
  for(unsigned i = 0; i < n; ++i)
    parameter_vector(i) = r_matrix(0, i);
 

  // for each component...
  for(unsigned m = 1; m < r_matrix.get_row_count(); ++m) {

    // temporarily this x vector is created
    // its simply holds the values of a specific component
    Vector<T> x(n);
    for(unsigned i = 0; i < n; ++i)
      x(i) = r_matrix(m, i);

 
    Matrix<T> coefficient_matrix(4, n - 1);
    Vector<T> h(n - 1);
    TriDiagonalMatrix<T> mat_c(n);
 
    
    // copy the function values into the coefficient matrix
    for(unsigned i = 0; i < n - 1; ++i)
      coefficient_matrix(0, i) = r_matrix(m, i);
    

    // calculate the h's
    for(unsigned i = 0; i < n - 1; ++i)
      h(i) = parameter_vector(i + 1) - parameter_vector(i);

    
    Vector<T>               r(n);
    
    //mat_c(0, n - 1) = h(n - 1);
    mat_c(0, 0)     = 2 * h(0);
    mat_c(0, 1)     = h(0);
    r(0)            = 3 * (x(1) - x(0)) / h(0);

    for(unsigned i = 1; i < n - 1; ++i) {
      mat_c(i, i - 1) = h(i - 1);
      mat_c(i, i    ) = 2 * (h(i - 1) + h(i));
      mat_c(i, i + 1) = h(i);
      
      r(i) = 3 * (x(i + 1) - x(i)) / h(i) - 3 * (x(i) - x(i - 1)) / h(i - 1);
    }
    
    mat_c(n - 1, n - 2) = h(n - 2);
    mat_c(n - 1, n - 1) = 2 * h(n - 2);

    r(n - 1) = - 3 * (x(n - 1) - x(n - 2)) / h(n - 2);
    
    // calculate the c's
    Vector<T> c(solve(mat_c, r));

    // copy the c's into the coefficient matrix
    // of course it would be nice getting rid of having to copy here
    for(unsigned i = 0; i < n - 1; ++i)
      coefficient_matrix(2, i) = c(i);

    
    // calculate the b's    
    for(unsigned i = 0; i < n - 1; ++i)
      coefficient_matrix(1, i) = (x(i + 1) - x(i)) / h(i) - h(i) * (c(i + 1) + 2 * c(i)) / 3;

    // coefficient_matrix(3, n - 2) = (x(0) - x(n - 1)) / h(n - 1) - h(n) * (c(0) + 2 * c(n - 1)) / 3;
    
    
    // calculate the d's
    for(unsigned i = 0; i < n - 1; ++i)
      coefficient_matrix(3, i) = (c(i + 1) - c(i)) / 3 * h(i);
    //coefficient_matrix(4, n - 1) = (c(0) - c(n - 1)) / (3 * h(n - 1));
    

    *(coefficient_matrix_set.begin() + (m - 1)) = coefficient_matrix;
  }  

}


template<class T>
Vector<T> CubicSpline<T>::get_point(T parameter)
{
  if(parameter < parameter_vector(0))
    IndexOutOfBoundsException().raise();
  
  if(parameter > parameter_vector( parameter_vector.get_dimension() - 1 ))
    IndexOutOfBoundsException().raise();

  unsigned index = 0;
  while(parameter >= parameter_vector(index))
    ++index;

  --index;
    
  Vector<T> result( coefficient_matrix_set.size() );
  T delta_t = parameter - parameter_vector(index);

  for(unsigned i = 0; i < coefficient_matrix_set.size(); ++i) {
    Matrix<T> &r_coefficient_matrix = coefficient_matrix_set[i];
        
    result(i) = r_coefficient_matrix(0, index) + 
                r_coefficient_matrix(1, index) * delta_t + 
                r_coefficient_matrix(2, index) * delta_t * delta_t + 
                r_coefficient_matrix(3, index) * delta_t * delta_t * delta_t;
  }

  return result;
}



template<class T>
T CubicSpline<T>::get_distant_point_parameter(T parameter, T distance)
{
  Vector<T> start_point(get_point(parameter));
  Vector<T> end_point(coefficient_matrix_set.size());

  T square_distance = distance * distance;

  T s = parameter;
  for(;;) {
    end_point = get_point(s);
    
    if((end_point - start_point).square_length() >= square_distance)
      return s;

    s += 0.001;
    if(s > parameter_vector( parameter_vector.get_dimension() - 1))
      return s;
  }
  
}



}
}

#endif
