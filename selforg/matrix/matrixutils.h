/***************************************************************************
                          matrixutils.h  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/
// provides utils for matrix calculation using the GSL (gnu scientific library)

#ifndef MATRIXUTILS_H
#define MATRIXUTILS_H


#include "matrix.h"

/**
 * namespace for the matrix library
 */
namespace matrix{

  /**
     calculates the eigenvalues of the real and symmetric matrix m
     and returns them as a column vector in descending order
   */
  Matrix eigenValuesRealSym(const Matrix &m);

  /**
     calculates the eigenvalues and corresponding eigenvectors
     of the the real and symmetric matrix m.
     The eigenvalues are returned them as a column vector in descending order
     and the belonging
     eigenvectors are stored in the columns of the matrix eigenvectors.
   */
  bool eigenValuesVectorsRealSym(const Matrix &m, Matrix& eigenvalues, Matrix& eigenvectors);

  /**
     calculates the eigenvalues of the matrix m and returns them as a column vectors
     seperated to real and imaginary part in descending order.
   */
  bool eigenValues(const Matrix &m, Matrix& vals_real, Matrix& vals_imag);

  /**
     calculates the eigenvalues and corresponding eigenvectors
     of the matrix m. The eigenvalues are returned them as a column vector
     in descending order and the belonging
     eigenvectors are stored in the columns of the matrix eigenvectors.
     Both are seperated into real and imaginary part.
   */
  bool eigenValuesVectors(const Matrix &m, Matrix& vals_real, Matrix& vals_imag,
                          Matrix& vecs_real, Matrix& vecs_imag);


  /**
     flips the signs of the eigenvectors such that their first entry has positive real part.
     If the first entry is very small the first no-vanishing entry is used.
     The eigenvectors are assumed to be columnwise (as returned by eigenValues() etc).
     Returns the original signs (1 or -1 per column)
   */
  std::vector<int> toPositiveSignEigenVectors(Matrix& vecs_real, Matrix& vecs_imag);

  /**
     scales the eigenvectors with the absolute value of the eigenvalues.
     returns vector with factors
   */
  Matrix scaleEigenVectorsWithValue(const Matrix& vals_real, const Matrix& vals_imag,
                                  Matrix& vecs_real, Matrix& vecs_imag);

}

#endif
