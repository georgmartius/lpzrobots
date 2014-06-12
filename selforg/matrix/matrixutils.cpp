/***************************************************************************
                          matrixutils.cpp  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/

#include "matrixutils.h"
#include <cmath>

#ifndef NO_GSL
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_eigen.h>

namespace matrix {
  /// copies our matrix into a gsl matrix (don't forget gsl_matrix_free)
  gsl_matrix* toGSL(const Matrix& src);
  /// copies gsl matrix to our matrix
  Matrix fromGSL(const gsl_matrix* src);
  /// copies gsl vector to our matrix (column-vector)
  Matrix fromGSL(const gsl_vector* src);
  /// copies the real part of gsl matrix_complex to our matrix
  Matrix fromGSL_real(const gsl_matrix_complex* src);
  /// copies the imag part of gsl matrix_complex to our matrix
  Matrix fromGSL_imag(const gsl_matrix_complex* src);

  Matrix eigenValuesRealSym(const Matrix &m){
    // check if m is square
    assert(m.getM() == m.getN());
    gsl_matrix* m_gsl = toGSL(m);
    gsl_set_error_handler_off();

    gsl_eigen_symm_workspace* w = gsl_eigen_symm_alloc (m.getM());
    gsl_vector *eval = gsl_vector_alloc (m.getM());

    gsl_eigen_symm (m_gsl, eval,  w);
    gsl_eigen_symm_free (w);

    // the multiplication with -1 causes a descending ordering
    Matrix m_eval = fromGSL(eval) * -1 ;
    m_eval.toSort();
    gsl_matrix_free(m_gsl);
    gsl_vector_free(eval);
    return m_eval * -1;
  }

  bool eigenValuesVectorsRealSym(const Matrix &m, Matrix& eigenvalues, Matrix& eigenvectors){
    // check if m is square
    assert(m.getM() == m.getN());
    gsl_matrix* m_gsl = toGSL(m);
    gsl_set_error_handler_off();

    gsl_eigen_symmv_workspace* w = gsl_eigen_symmv_alloc (m.getM());
    gsl_vector *eval = gsl_vector_alloc (m.getM());
    gsl_matrix *evec = gsl_matrix_alloc (m.getM(), m.getM());

    gsl_eigen_symmv (m_gsl, eval, evec, w);
    gsl_eigen_symmv_free (w);
    gsl_eigen_symmv_sort (eval, evec,
                          GSL_EIGEN_SORT_ABS_DESC);
    eigenvalues = fromGSL(eval);
    eigenvectors = fromGSL(evec);
    gsl_matrix_free(m_gsl);
    gsl_vector_free(eval);
    gsl_matrix_free(evec);
    return true;
  }

  // check for exception:
  // gsl: francis.c:210: ERROR: maximum iterations reached without finding all eigenvalues
  //Default GSL error handler invoked.
  bool eigenValues(const Matrix &m, Matrix& real, Matrix& imag){
    // check if m is square
    assert(m.getM() == m.getN());
    gsl_matrix* m_gsl = toGSL(m);
    gsl_set_error_handler_off();

    gsl_eigen_nonsymm_workspace* w = gsl_eigen_nonsymm_alloc (m.getM());
    gsl_vector_complex *eval = gsl_vector_complex_alloc (m.getM());

    gsl_eigen_nonsymm (m_gsl, eval,  w);
    gsl_eigen_nonsymm_free (w);
    gsl_eigen_nonsymmv_sort (eval, 0, GSL_EIGEN_SORT_ABS_DESC);
    gsl_vector_view eval_real = gsl_vector_complex_real(eval);
    gsl_vector_view eval_imag = gsl_vector_complex_imag(eval);
    real = fromGSL(&eval_real.vector);
    imag = fromGSL(&eval_imag.vector);
    gsl_matrix_free(m_gsl);
    gsl_vector_complex_free(eval);
    return true;
  }

  bool eigenValuesVectors(const Matrix &m, Matrix& vals_real, Matrix& vals_imag,
                          Matrix& vecs_real, Matrix& vecs_imag){
    // check if m is square
    assert(m.getM() == m.getN());
    gsl_matrix* m_gsl = toGSL(m);
    gsl_set_error_handler_off();

    gsl_eigen_nonsymmv_workspace* w = gsl_eigen_nonsymmv_alloc (m.getM());
    gsl_vector_complex *eval = gsl_vector_complex_alloc (m.getM());
    gsl_matrix_complex *evec = gsl_matrix_complex_alloc (m.getM(), m.getM());

    gsl_eigen_nonsymmv (m_gsl, eval, evec, w);
    gsl_eigen_nonsymmv_free (w);
    gsl_eigen_nonsymmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_DESC);
    gsl_vector_view eval_real = gsl_vector_complex_real(eval);
    gsl_vector_view eval_imag = gsl_vector_complex_imag(eval);
    vals_real = fromGSL(&eval_real.vector);
    vals_imag = fromGSL(&eval_imag.vector);
    vecs_real = fromGSL_real(evec);
    vecs_imag = fromGSL_imag(evec);

    gsl_matrix_free(m_gsl);
    gsl_vector_complex_free(eval);
    gsl_matrix_complex_free(evec);
    return true;

  }


/******************** local functions *************************************/

  gsl_matrix* toGSL(const Matrix& src){
    gsl_matrix * m = gsl_matrix_alloc (src.getM(), src.getN());
    assert(m);
    assert(sizeof(double) == sizeof(D));
    // if the tda (row length) is equal to N (size2) then we can copy the data right away
    if(m->tda == m->size2){
      memcpy(m->data, src.unsafeGetData(), m->size1*m->size2* sizeof(D));
    }else{ // otherwise we have to copy it rowwise
      for (unsigned int i = 0; i < m->size1; i++){
        memcpy(m->data + i*m->tda, src.unsafeGetData() + i*src.getN(),
               m->size2 * sizeof(D));
      }
    }
    return m;
  }


  Matrix fromGSL(const gsl_matrix* src){
    assert(src);
    Matrix m;
    // if the tda (row length) is equal to N (size2) then we can copy the data right away
    if(src->tda == src->size2){
      m.set(src->size1, src->size2, src->data);
    }else{
      m.set(src->size1, src->size2);
      for (unsigned int i = 0; i < src->size1; i++){
        memcpy(&m.val(i,0), src->data + i*src->tda,
               src->size2 * sizeof(D));
      }

    }
    return m;
  }

  Matrix fromGSL(const gsl_vector* src){
    assert(src);
    Matrix m;
    // if the stride == 1 (stepsize in memory) we can copy the data right away
    if(src->stride == 1){
      m.set(src->size, 1, src->data);
    }else{ // copy all elements one by one (slow)
      m.set(src->size, 1);
      for (unsigned int i = 0; i < src->size; i++){
        m.val(i,0) = gsl_vector_get(src,i);
      }
    }
    return m;
  }

  Matrix fromGSL_real(const gsl_matrix_complex* src){
    assert(src);
    Matrix m(src->size1, src->size2);
    for (unsigned int i = 0; i < src->size1; i++){
      for (unsigned int j = 0; j < src->size2; j++){
        m.val(i,j) = GSL_REAL(gsl_matrix_complex_get(src, i, j));
      }
    }
    return m;
  }

  Matrix fromGSL_imag(const gsl_matrix_complex* src){
    assert(src);
    Matrix m(src->size1, src->size2);
    for (unsigned int i = 0; i < src->size1; i++){
      for (unsigned int j = 0; j < src->size2; j++){
        m.val(i,j) = GSL_IMAG(gsl_matrix_complex_get(src, i, j));
      }
    }
    return m;
  }
}
#else /// NO GSL - implement dummy versions

namespace matrix {
  Matrix eigenValuesRealSym(const Matrix &m){
    assert("Not implemented!");
    return Matrix();
  }

  bool eigenValuesVectorsRealSym(const Matrix &m, Matrix& eigenvalues,
                                 Matrix& eigenvectors){
    assert("Not implemented!");
    return false;
  }

  bool eigenValues(const Matrix &m, Matrix& real, Matrix& imag){
    assert("Not implemented!");
    return false;
  }

  bool eigenValuesVectors(const Matrix &m, Matrix& vals_real, Matrix& vals_imag,
                          Matrix& vecs_real, Matrix& vecs_imag){
    assert("Not implemented!");
    return false;
  }

}

#endif

namespace matrix {

  std::vector<int> toPositiveSignEigenVectors(Matrix& vecs_real, Matrix& vecs_imag){
    std::vector<int> signs(vecs_real.getN());
    for(unsigned int i=0; i< vecs_real.getN(); i++){
      unsigned int k=0;
      if(fabs(vecs_real.val(0,i))<0.1) {
        while(fabs(vecs_real.val(k,i))<0.1 && k<vecs_real.getN()-1) k++;
      }
      if(vecs_real.val(k,i)<0) {
        signs[i]=-1;
        for(unsigned int j=0; j < vecs_real.getM(); j++){
          vecs_real.val(j,i)*=-1;
          vecs_imag.val(j,i)*=-1;
        }
      }else{
        signs[i]=1;
      }
    }
    return signs;
  }

  double __matutils_euklidlen(double a, double b) { return sqrt(a*a+b*b);}

  Matrix scaleEigenVectorsWithValue(const Matrix& vals_real, const Matrix& vals_imag,
                                  Matrix& vecs_real, Matrix& vecs_imag){
    const Matrix& factors = Matrix::map2(__matutils_euklidlen, vals_real, vals_imag);
    vecs_real= ((vecs_real^T)&factors)^T;
    vecs_imag= ((vecs_imag^T)&factors)^T;
    return factors;
  }

}
