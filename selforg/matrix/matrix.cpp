/***************************************************************************
                          matrix.cpp  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/
// 
// $Log$
// Revision 1.12  2007-08-22 08:27:58  martius
// contrains for reshape relaxed
//
// Revision 1.11  2007/06/21 16:29:27  martius
// added map2P
// map2 into cpp
//
// Revision 1.10  2007/05/22 13:52:46  martius
// inplace operators return *this which makes them more useable for temporary matrices
//
// Revision 1.9  2007/04/03 09:57:44  martius
// speedup in nonoptimised mode through use of VAL macro
//
// Revision 1.8  2007/04/03 07:11:56  der
// plus lambdaI
//
// Revision 1.7  2007/02/05 12:31:21  martius
// reshape
//
// Revision 1.6  2006/11/29 09:57:53  martius
// bugfix in rows!
//
// Revision 1.5  2006/08/04 15:16:13  martius
// documentation
//
// Revision 1.4  2006/07/20 17:14:35  martius
// removed std namespace from matrix.h
// storable interface
// abstract model and invertablemodel as superclasses for networks
//
// Revision 1.3  2006/07/19 09:26:28  martius
// namespace std removed from header
// store and restore
// read and write
// columns accessor
//
// Revision 1.2  2006/07/14 12:24:01  martius
// selforg becomes HEAD
//
// Revision 1.1.2.2  2006/07/14 08:57:40  der
// New function isNullTimesNull
//
// Revision 1.1.2.1  2006/07/10 12:01:01  martius
// Matrixlib moved to selforg
//
// Revision 1.23.6.1  2006/03/29 15:12:46  martius
// column accessor function added
//
// Revision 1.23  2005/10/21 11:58:25  martius
// map2 (similar to map but for 2 matrices)
// changed naming of functions to be more consistent.
//  Functions with "to" in front indicate the change of this. (Still not consistent with add, mult ...)
//
// Revision 1.22  2005/10/06 17:10:06  martius
// convertToList
// above and toAbove
//
// Revision 1.21  2005/09/21 08:42:53  martius
// convertToBuffer is const
//
// Revision 1.20  2005/08/06 20:47:36  martius
// Commented
//
// Revision 1.19  2005/07/21 15:13:36  martius
// mapP addid (mapping with additional parameter)
//
// Revision 1.18  2005/07/07 10:26:59  martius
// exp(1x1) matrix implemented
//
// Revision 1.17  2005/06/21 15:35:21  martius
// hide invert3x3 and invert_nonzero for AVR to minimize binary size
//
// Revision 1.16  2005/06/19 23:00:30  martius
// AVR
//
// Revision 1.15  2005/06/17 15:19:03  martius
// version workes with avr
//
// Revision 1.14  2005/06/10 08:21:59  martius
// mult???wise are copy operations now!
// toMult???wise are inplace instead
//
// Revision 1.13  2005/06/10 07:50:47  martius
// multMT and multTM are const!
//
// Revision 1.12  2005/06/09 11:52:03  martius
// multMT (M * M^T) and multTM (M^T * M)
//
// Revision 1.11  2005/06/02 22:48:48  martius
// copy is inline and works correct now
//
// Revision 1.10  2005/06/02 08:49:08  martius
// mult_row/column_wise
// convertToBuffer
// Matrix comparison uses epsilon instead of plain ==
//
// Revision 1.9  2005/05/30 22:40:56  martius
// map becomes toMap and the new map returns a new matrix
// exp becomes toExp
//
// Revision 1.7  2005/05/30 17:21:41  martius
// added zero
// set() and constructor(m,n,0) initialise with zero
// id returns void (more consistent)
//
// Revision 1.6  2005/05/30 16:43:12  martius
// map function included (component-wise function application)
//
// Revision 1.5  2005/05/30 10:15:36  martius
// proper log entry in header
//
/***************************************************************************/

#include "matrix.h"
#include <string.h>
#include <math.h>
#include <algorithm>

namespace matrix {

#define COMPARE_EPS 1e-12
#define VAL(i,j) data[i*n+j]

const int T=0xFF;


  Matrix::Matrix(const Matrix& c)
    : m(0), n(0), buffersize(0), data(0){
    copy(c);
  }
  
  Matrix::Matrix(unsigned short _m, unsigned short _n, const D* _data /*=0*/)
    : m(_m), n(_n), buffersize(0), data(0) {
    allocate(); 
    set(_data);
  };

  // internal allocation
  void Matrix::allocate() {
    if((unsigned)m*n > buffersize){
      buffersize=m*n;      
      if(data) { free(data); }
      data = (D*) malloc(sizeof(D)*buffersize);
      assert(data);
    }
  }

////////////////////////////////////////////////////////////////////////////////
////// ACCESSORS ///////////////////////////////////////////////////////////////
  /// returns true if matrix is a 0x0 matrix
  bool Matrix::isNulltimesNull(){
    return (m==0 && n==0);
  }


  Matrix Matrix::row(unsigned short index) const{
    assert(index < m);
    Matrix result(1,n,data+(index*n));    
    return result;
  }

  Matrix Matrix::rows(unsigned short startindex, unsigned short endindex) const{
    unsigned short start = std::min( (int)startindex, m-1 );
    unsigned short end   = std::max( (int)start, std::min((int)endindex,m-1));
    unsigned short k     = end - start + 1;
    Matrix result(k,n);
    memcpy(result.data, data + start*n, k*n*sizeof(D));    
    return result;
  }

  Matrix Matrix::column(unsigned short index) const {
    assert(index < n);    
    Matrix result(m,1);    
    for(int i=0; i<m; i++){
      result.val(i,0) = VAL(i,index);
    }
    return result;
  }

  Matrix Matrix::columns(unsigned short startindex, unsigned short endindex) const{
    unsigned short start = std::min( (int)startindex, n-1 );
    unsigned short end   = std::max( (int)start, std::min((int)endindex,n-1));
    unsigned short k     = end - start + 1;
    Matrix result(m,k);
    for(int i=0; i<m; i++){
      memcpy(result.data + i*k, data + i*n + start, k*sizeof(D));
    }
    return result;
  }

  void Matrix::set(unsigned short _m, unsigned short _n, const D* _data /*=0*/){
    m=_m;
    n=_n;
    allocate(); 
    set(_data);  
  }

  void Matrix::set(const D* _data){
    if (_data) 
      memcpy(data,_data, m*n*sizeof(D));
    else toZero();
  }

  int Matrix::convertToBuffer(D* buffer, unsigned int len) const {
    if(buffer && data){
      unsigned int minlen = (len < (unsigned)m*n) ? len : m*n; 
      memcpy(buffer, data, sizeof(D) * minlen);
      return minlen;
    }
    return 0;
  }

  std::list<D> Matrix::convertToList() const {
    std::list<D> l;
    if(data){      
      for(int i=0; i < m*n; i++){
	l.push_back(data[i]);
      }
    }
    return l;
  }
  
  bool Matrix::write(FILE* f) const {
    fprintf(f,"%i %i\n", m,n);
    for(int i=0; i< m*n; i++){
      fprintf(f,"%f ", data[i]);
    }
    fprintf(f,"\n");
    return true;
  }
  
  bool Matrix::read(FILE* f) {
    char buffer[128];
    if(fscanf(f,"%hu %hu\n", &m,&n)!=2)  return false;
    allocate();
    for(int i=0; i< m*n; i++){
      if(fscanf(f,"%s ", buffer)!=1) return false;
      data[i] = atof(buffer);
    }
    fscanf(f,"\n");
    return true;
  }

  /** stores the Matrix into the given file stream (binary)
   */
  bool Matrix::store(FILE* f) const {
    int dim[2] = { m, n };
    unsigned int len = m * n;
    bool rval=false;
    if(fwrite(dim, sizeof(int), 2, f) == 2) 
      if(fwrite(data, sizeof(D), len, f) == len)       
	rval=true;
    return rval;
  }
  
  /** reads a Matrix from the given file stream (binary)
   */
  bool Matrix::restore(FILE* f) {
    int dim[2];
    bool rval = false;
    if(fread(dim, sizeof(int), 2, f) == 2){
      m=dim[0]; 
      n=dim[1];
      allocate();
      unsigned int len = m * n;      
      if(fread(data, sizeof(D), len, f) == len) {  
	rval = true;
      }else fprintf(stderr, "Matrix::restore: cannot read matrix data\n"); 
    }else{
      fprintf(stderr, "Matrix::restore: cannot read dimension\n");
    }
    return rval;
  }
  
  

////////////////////////////////////////////////////////////////////////////////
////// OPERATORS ///////////////////////////////////////////////////////////////
//  INPLACE:
//________________________________________________________________
  
  Matrix& Matrix::toTranspose(){
    assert(buffersize > 0);
    if(m!=1 && n!=1){ // if m or n == 1 then no copying is necessary! 
      double* newdata = (D*)malloc(sizeof(D)*buffersize);
      for(unsigned short i=0; i < m; i++){
	for(unsigned short j=0; j < n; j++){
	  newdata[j*m+i]=data[i*n+j];
	}
      }
      free(data);
      data=newdata;    
    }
    // swap n and m
    unsigned short t = m;
    m=n;
    n=t;
    return *this;
  }

  Matrix& Matrix::toZero(){
    memset(data, D_Zero, m*n*sizeof(D));   
    return *this;
  }

  Matrix& Matrix::toId(){
    toZero();
    unsigned short smallerdim = m < n ? m : n;
    for(unsigned short i=0; i < smallerdim; i++){
      VAL(i,i)=D_One;
    }
    return *this;    
  }

  void Matrix::add(const Matrix& a,const Matrix& b){
    assert(a.m==b.m && a.n==b.n);
    copy(a);
    toSum(b);
  }

  void Matrix::sub(const Matrix& a,const Matrix& b){
    assert(a.m==b.m && a.n==b.n);
    copy(a);
    toDiff(b);
  }

  void Matrix::mult(const Matrix& a,const Matrix& b){
    assert(a.n==b.m);
    m=a.m;
    n=b.n;    
    unsigned short interdim = a.n;
    allocate();
    D d;
    for(unsigned short i=0; i < m; i++){
      for(unsigned short j=0; j < n; j++){
	d=0;
	for(unsigned short k=0; k < interdim; k++){
	  d+=a.val(i,k)*b.val(k,j);
	}
	VAL(i,j)=d;	
      }
    }
  }

  void Matrix::mult(const Matrix& a, const D& fac){
    m=a.m;
    n=a.n;    
    allocate();
    for(unsigned short i=0; i<m*n; i++){
      data[i]=a.data[i]*fac;
    }    
  }

  Matrix& Matrix::toMult(const D& fac){
    for(unsigned short i=0; i<m*n; i++){
      data[i]*=fac;
    }    
    return *this;
  }

  /** special inplace matrix potence: 
      @param: exp -1 -> inverse; 0 -> Identity Matrix; 
      1 -> itself;
      T -> Transpose
  */
  Matrix& Matrix::toExp (int exponent) {
    switch(exponent){
    case -1: 
      if(m==1) VAL(0,0) = 1/VAL(0,0);
      else if(m==2) invert2x2();
#ifndef AVR      
      else if(m==3) invert3x3();
      else invertnonzero();
#endif
      break;
    case 0:
      toId();
      break;
    case 1: // do nothing
      break;
    case T:
      toTranspose();
      break;
    default:
      assert("Should not be reached" == 0);
      break;
    }
    return *this;
  }

  Matrix& Matrix::toMap(D (*fun)(D)){
    unsigned int len = m*n;
    for(unsigned short i=0; i < len; i++){
      data[i]=fun(data[i]);
    }    
    return *this;
  }

  Matrix Matrix::map(D (*fun)(D)) const {
    Matrix result(*this);
    result.toMap(fun);
    return result;
  }

  Matrix& Matrix::toMapP(void* param, D (*fun)(void*, D)){
    unsigned int len = m*n;
    for(unsigned short i=0; i < len; i++){
      data[i]=fun(param, data[i]);
    }    
    return *this;
  }

  Matrix Matrix::mapP(void* param, D (*fun)(void*, D)) const {
    Matrix result(*this);
    result.toMapP(param,fun);
    return result;
  }

  Matrix& Matrix::toMultrowwise(const Matrix& factors){
    assert(m == factors.m && factors.n == 1);
    for(unsigned int i = 0; i < m; i++){
      for (unsigned int j = 0; j < n; j++){
	VAL(i,j) *= factors.val(i,0);
      }
    }
    return *this;
  }

  Matrix& Matrix::toMultcolwise(const Matrix& factors){
    assert(n == factors.m && factors.n == 1);
    for(unsigned int i = 0; i < m; i++){
      for (unsigned int j = 0; j < n; j++){
	VAL(i,j) *= factors.val(j,0);
      }
    }
    return *this;
  }

  Matrix& Matrix::toAbove(const Matrix& a){
    assert(a.n == this->n);
    data = (D*)realloc(data, sizeof(D) * (this->m * this->n + a.n * a.m));
    memcpy(data+this->m * this->n, a.data, sizeof(D) * (a.n * a.m));
    this->m+=a.m; 
    return *this;      
  }

  int cmpdouble(const void* a, const void* b){
    return *((double*)a) < *((double*)b) ? -1 : (*((double*)a) > *((double*)b) ? 1 : 0);
  }
  
  Matrix& Matrix::toSort(){
    qsort(data, m*n, sizeof(double), cmpdouble); 
    return *this;
  }

  Matrix& Matrix::reshape(int _m, int _n){
    assert(_m*_n <= m*n);
    m=_m;
    n=_n;
    return *this;
  }

  /// adds the given value to the diagonal
  Matrix& Matrix::pluslambdaI(double lambda){
    int smallerdim=std::min(m,n);
    for(int i=0; i< smallerdim; i++){
      VAL(i,i)+=lambda;
    }
    return *this;
  }

  Matrix Matrix::map2( D (*fun)(D,D), const Matrix& a, const Matrix& b) {
    assert(a.m == b.m && a.n == b.n);
    Matrix result(a);
    unsigned int len = a.m*a.n;
    for(unsigned short i=0; i < len; i++){
      result.data[i] = fun(a.data[i], b.data[i]);
    }    
    return result;
  }

  Matrix Matrix::map2P( void* param, D (*fun)(void*, D,D), const Matrix& a, const Matrix& b){
    assert(a.m == b.m && a.n == b.n);
    Matrix result(a);
    unsigned int len = a.m*a.n;
    for(unsigned short i=0; i < len; i++){
      result.data[i] = fun(param, a.data[i], b.data[i]);
    }    
    return result;
  }

  Matrix Matrix::multrowwise(const Matrix& factors) const {
    Matrix result(*this);
    result.toMultrowwise(factors);
    return result;
  }

  Matrix Matrix::multcolwise(const Matrix& factors) const{
    Matrix result(*this);
    result.toMultcolwise(factors);
    return result;
  }

  // multiply Matrix with its transposed: M * M^T
  Matrix Matrix::multMT() const {
    assert(m != 0 && n != 0);
    Matrix result(m,m);
    D d;
    for(unsigned short i=0; i < m; i++){
      for(unsigned short j=0; j < m; j++){
	d=0;
	for(unsigned short k=0; k < n; k++){
	  d+=VAL(i,k) * VAL(j,k);
	}
	result.val(i,j)=d;	
      }
    }
    return result;
  }

  // multiply transpsoed of Matrix with itself: M^T * M
  Matrix Matrix::multTM() const {
    assert(m != 0 && n != 0);
    Matrix result(n,n);
    D d;
    for(unsigned short i=0; i < n; i++){
      for(unsigned short j=0; j < n; j++){
	d=0;
	for(unsigned short k=0; k < m; k++){
	  d+=VAL(k,i) * VAL(k,j);
	}
	result.val(i,j)=d;	
      }
    }
    return result;
  }

  /// returns the product of all elements
  D Matrix::elementProduct() const {
    D rv=1;
    for(unsigned short i=0; i<m*n; i++){
      rv*=data[i];
    }
    return rv;
  }

  /// returns the sum of all elements
  D Matrix::elementSum() const {
    D rv=0;
    for(unsigned short i=0; i<m*n; i++){
      rv+=data[i];
    }
    return rv;
  }

  /// returns a matrix that consists of b below this
  Matrix Matrix::above(const Matrix& b) const {
    Matrix r(*this);
    r.toAbove(b);
    return r;
  }


#ifndef AVR  
  /* inplace matrix invertation:
      Matrix must be SQARE, in addition, all DIAGONAL ELEMENTS MUST BE NONZERO  */  
  void Matrix::invertnonzero()  {
    assert(m==n && m>1); // must be of dimension >= 2

    for (unsigned int i=1; i < m; i++) data[i] /= data[0]; // normalize row 0
    for (unsigned int i=1; i < m; i++)  { 
      for (unsigned int j=i; j < m; j++)  { // do a column of L
        D sum = 0.0;
        for (unsigned int k = 0; k < i; k++)  
	  sum += VAL(j,k) * VAL(k,i);
        VAL(j,i) -= sum;
      }
      if (i == (unsigned) m-1) continue;
      for (unsigned int j=i+1; j < m; j++)  {  // do a row of U
        D sum = 0.0;
        for (unsigned int k = 0; k < i; k++)
	  sum += VAL(i,k)*VAL(k,j);
        VAL(i,j) = (VAL(i,j)-sum) / VAL(i,i);
      }
    }
    for (unsigned int i = 0; i < m; i++ )  // invert L
      for (unsigned int j = i; j < m; j++ )  {
        D x = 1.0;
        if ( i != j ) {
          x = 0.0;
          for (unsigned int k = i; k < j; k++ ) 
	    x -= VAL(j,k)*VAL(k,i);
	}
        VAL(j,i) = x / VAL(j,j);
      }
    for (unsigned int i = 0; i < m; i++ )   // invert U
      for (unsigned int j = i; j < m; j++ )  {
        if ( i == j ) continue;
        D sum = 0.0;
        for (unsigned int k = i; k < j; k++ )
	  sum += VAL(k,j)*( (i==k) ? 1.0 : VAL(i,k) );
        VAL(i,j) = -sum;
      }
    for (unsigned int i = 0; i < m; i++ )   // final inversion
      for (unsigned int j = 0; j < m; j++ )  {
        D sum = 0.0;
        for (unsigned int k = ((i>j)?i:j); k < m; k++ )  
	  sum += ((j==k)?1.0:VAL(j,k))*VAL(k,i);
        VAL(j,i) = sum;
      }
  };
#endif
  

  void Matrix::invert2x2(){
    assert(m==n && m==2);   
    // high speed version
    D detQ = data[0] * data[3] - data[1] * data[2];    
    D tmp = data[0];
    data[0] = data[3] / detQ;
    data[3] = tmp / detQ;
    data[1] /= -detQ;
    data[2] /= -detQ;
  }

#ifndef AVR
  void Matrix::invert3x3(){
    assert(m==n && m==3);
    D Q_adjoint[3][3];
    D detQ=0;
    //calculate the inverse of Q
    Q_adjoint[0][0]=VAL(1,1)*VAL(2,2)-VAL(1,2)*VAL(2,1) ;
    Q_adjoint[0][1]=(VAL(1,2)*VAL(2,0)-VAL(1,0)*VAL(2,2)) ;
    Q_adjoint[0][2]=VAL(1,0)*VAL(2,1)-VAL(1,1)*VAL(2,0) ;
    Q_adjoint[1][0]=(VAL(2,1)*VAL(0,2)-VAL(0,1)*VAL(2,2)) ;
    Q_adjoint[1][1]=VAL(0,0)*VAL(2,2)-VAL(0,2)*VAL(2,0) ;
    Q_adjoint[1][2]=(VAL(0,1)*VAL(2,0)-VAL(0,0)*VAL(2,1)) ;
    Q_adjoint[2][0]=VAL(0,1)*VAL(1,2)-VAL(1,1)*VAL(0,2) ;
    Q_adjoint[2][1]=(VAL(1,0)*VAL(0,2)-VAL(0,0)*VAL(1,2)) ;
    Q_adjoint[2][2]=VAL(0,0)*VAL(1,1)-VAL(0,1)*VAL(1,0) ;
    detQ=VAL(0,0)*Q_adjoint[0][0]+VAL(0,1)*Q_adjoint[0][1]+VAL(0,2)*Q_adjoint[0][2] ;
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++) {                                                     
	VAL(i,j)=(Q_adjoint[j][i])/detQ  ;
      }
    }          
  }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  // normal binary operators

  Matrix Matrix::operator +  (const Matrix& sum) const {
    Matrix result;
    result.add(*this, sum);
    return result;
  }
  Matrix Matrix::operator -  (const Matrix& sum) const {
    Matrix result;
    result.sub(*this, sum);
    return result;
  }
  /** matrix product*/
  Matrix Matrix::operator *  (const Matrix& fac) const {
    Matrix result;
    result.mult(*this, fac);    
    return result;
  }
  /** product with scalar (double)*/
  Matrix Matrix::operator *  (const D& scalar) const {
    Matrix result;
    result.mult(*this, scalar);
    return result;
  }
  /** special matrix potence: 
      @param: exp -1 -> inverse; 0 -> Identity Matrix; 
      1 -> itself; 2-> Matrix*Matrix^T
      T -> Transpose
  */
  Matrix Matrix::operator ^ (int exponent) const {
    Matrix result(*this);
    result.toExp(exponent);        
    return result;
  }

#ifndef AVR
  bool Matrix::operator == (const Matrix& c) const {
    if(m!=c.m || n!=c.n) return false;
    D* p1=data;
    D* p2=c.data;
    unsigned int len = m*n;
    for(unsigned int i = 0; i < len; i++){
      if(fabs(*p1 - *p2) > COMPARE_EPS) {
	return false;
      }
      p1++; p2++;
    }
    return true;
  }

  std::ostream& operator<<(std::ostream& str, const Matrix& mat){
    if (mat.m==0 || mat.n==0) return str << "0";
    else { 
      //      str << mat.m << "x" << mat.n << " (\n";
      for(int i=0; i < mat.m; i++){
        for(int j=0; j < mat.n; j++){
          str << mat.val(i,j) << " \t"; 
        }
        if(i!=mat.m-1) str << std::endl;
      }      
    }
    //    return str << ")\n";
    return str;
  }
#endif

}

#include "matrix.tests.hpp"
