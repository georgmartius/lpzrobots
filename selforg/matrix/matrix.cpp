/***************************************************************************
                          matrix.cpp  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/
// provides Matrix class with convinient operators
//  and fast inversion for nonzero square matrixes
//
/***************************************************************************/

#include "matrix.h"
#include <string.h>
#include <cmath>
#include <algorithm>

namespace matrix {

#define COMPARE_EPS 1e-12
#define VAL(i,j) data[i*n+j]

  const int T = 0xFF;


  Matrix::Matrix ( const Matrix& c )
      : m ( 0 ), n ( 0 ), buffersize ( 0 ), data ( 0 ) {
    copy ( c );
  }

  Matrix::Matrix ( Matrix&& c )
    : m ( c.m ), n ( c.n ), buffersize ( c.buffersize ), data ( c.data ) {
    c.data=0;
    c.buffersize=0;
  }

  Matrix::Matrix ( I _m, I _n, const D* _data /*=0*/ )
      : m ( _m ), n ( _n ), buffersize ( 0 ), data ( 0 ) {
    allocate();
    set ( _data );
  };
  Matrix::Matrix ( I _m, I _n, D def)
    : m ( _m ), n ( _n ), buffersize ( 0 ), data ( 0 ) {
    allocate();
    for(I i=0; i<buffersize; i++){
      data[i]=def;
    }
  };

  Matrix& Matrix::operator = (Matrix&&c){
    if(data) free(data);
    m=c.m; n=c.n; buffersize=c.buffersize; data=c.data;
    c.data=0; c.buffersize=0;
    return *this;
  }

  // internal allocation
  void Matrix::allocate()
  {
    if (m*n == 0)
    {
      if (data)
        free(data);
      data = 0;
      buffersize = 0;
    }

    if ( m*n > buffersize )
    {
      buffersize = m * n;
      if ( data )
        free ( data );

      data = ( D* ) malloc ( sizeof ( D ) * buffersize );
      assert ( data );
    }
  }

////////////////////////////////////////////////////////////////////////////////
////// ACCESSORS ///////////////////////////////////////////////////////////////
  /// returns true if matrix is a 0x0 matrix
  bool Matrix::isNulltimesNull() const {
    return ( m == 0 && n == 0 );
  }

  /// returns true if matrix is a vector
  bool Matrix::isVector() const {
    return ( m == 1 || n == 1 );
  }

#ifdef WIN32
  int bcmp (const void *_a, const void *_b, int count){
    int rtnval = 0;
    const char* a = (char*) _a;
    const char* b = (char*) _b;
    while (count-- > 0) {
      if (*a++ != *b++) {
        rtnval = 1;
        break;
      }
    }
    return (rtnval);
  }
#endif

  bool Matrix::equals (const Matrix& a) const {
    if(m*n != a.m*a.n) return false;
    return (bcmp(data,a.data, sizeof(double)*m*n) == 0);
  }


  Matrix Matrix::row ( I index ) const {
    assert ( index < m );
    Matrix result ( 1, n, data + ( index*n ) );
    return result;
  }

  Matrix Matrix::rows ( I startindex, I endindex ) const {
    assert(startindex<=endindex);
    I start = (I)std::min ( ( int ) startindex, (int) m - 1 );
    I end   = (I)std::max ( ( int ) start, std::min ( ( int ) endindex, (int) m - 1 ) );
    I k     = end - start + 1;
    if ( k == m ) return *this;
    Matrix result ( k, n );
    memcpy ( result.data, data + start*n, k*n*sizeof ( D ) );
    return result;
  }

  Matrix Matrix::column ( I index ) const {
    assert ( index < n );
    Matrix result ( m, 1 );
    for ( I i = 0; i < m; i++ ) {
      result.val ( i, 0 ) = VAL ( i, index );
    }
    return result;
  }

  Matrix Matrix::columns ( I startindex, I endindex ) const {
    I start = std::min ( ( I ) startindex, n - 1 );
    I end   = std::max ( ( I ) start, std::min ( ( I ) endindex, n - 1 ) );
    I k     = end - start + 1;
    Matrix result ( m, k );
    for ( I i = 0; i < m; i++ ) {
      memcpy ( result.data + i*k, data + i*n + start, k*sizeof ( D ) );
    }
    return result;
  }

  void Matrix::set ( I _m, I _n, const D* _data /*=0*/ ) {
    m = _m;
    n = _n;
    allocate();
    set ( _data );
  }

  void Matrix::set ( const D* _data ) {
    if ( _data )
      memcpy ( data, _data, m*n*sizeof ( D ) );
    else toZero();
  }

  int Matrix::convertToBuffer ( D* buffer, I len ) const {
    if ( buffer && data ) {
      I minlen = ( len < ( I ) m * n ) ? len : m * n;
      memcpy ( buffer, data, sizeof ( D ) * minlen );
      return minlen;
    }
    return 0;
  }

  std::list<D> Matrix::convertToList() const {
    std::list<D> l;
    if ( data ) {
      for ( I i = 0; i < m*n; i++ ) {
        l.push_back ( data[i] );
      }
    }
    return l;
  }

  bool Matrix::write ( FILE* f ) const {
    fprintf ( f, "MATRIX %i %i\n", m, n );
    for ( I i = 0; i < m; i++ ) {
      for ( I j = 0; j < n; j++ ) {
        fprintf ( f, "%f ", VAL(i,j) );
      }
      fprintf ( f, "\n");
    }
    return true;
  }

  bool Matrix::read ( FILE* f, bool skipIdentifier ) {
    char buffer[128];
    if(!skipIdentifier){
      if ( fscanf ( f, "MATRIX %u %u\n", &m, &n ) != 2 )  return false;
    } else {
      if ( fscanf ( f, "%u %u\n", &m, &n ) != 2 )  return false;
    }
    allocate();
    for ( I i = 0; i < m*n; i++ ) {
      if ( fscanf ( f, "%s ", buffer ) != 1 ) return false;
      data[i] = atof ( buffer );
    }
    if( fscanf ( f, "\n" ) != 0) return false;
    return true;
  }

  /** stores the Matrix into the given file stream ASCII
   */
  bool Matrix::store ( FILE* f ) const {
    return write(f);
    // I dim[2] = { m, n };
    // I len = m * n;
    // bool rval = false;
    // if ( fwrite ( dim, sizeof ( I ), 2, f ) == 2 )
    //   if ( fwrite ( data, sizeof ( D ), len, f ) == len )
    //     rval = true;
    // return rval;
  }

  /** reads a Matrix from the given file stream ASCII (can load old binary format)
   */
  bool Matrix::restore ( FILE* f ) {
    char buffer[7];
    bool rval = false;
    if ( fread ( buffer, 7, 1, f ) == 1 ) {
      if ( buffer[0] == 'M' && buffer[1] == 'A' && buffer[2] == 'T'
           && buffer[3] == 'R' && buffer[4] == 'I' && buffer[5] == 'X' ) {
        return read ( f, true );
      }else{
        fseek ( f, -7, SEEK_CUR );
        I dim[2];
        if ( fread ( dim, sizeof ( I ), 2, f ) == 2 ) {
          m = dim[0];
          n = dim[1];
          allocate();
          I len = m * n;
          if ( fread ( data, sizeof ( D ), len, f ) == len ) {
            rval = true;
          } else fprintf ( stderr, "Matrix::restore: (binary) cannot read matrix data\n" );
        } else {
          fprintf ( stderr, "Matrix::restore: (binary) cannot read dimension\n" );
        }
      }
    }else{
      fprintf ( stderr, "Matrix::restore: cannot read MATRIX identifier/binary dimension\n" );
    }
    return rval;
  }


////////////////////////////////////////////////////////////////////////////////
////// OPERATORS ///////////////////////////////////////////////////////////////
//  INPLACE:
//________________________________________________________________

  Matrix& Matrix::toTranspose() {
    assert ( buffersize > 0 );
    if ( m != 1 && n != 1 ) { // if m or n == 1 then no copying is necessary!
      double* newdata = ( D* ) malloc ( sizeof ( D ) * buffersize );
      for ( I i = 0; i < m; i++ ) {
        for ( I j = 0; j < n; j++ ) {
          newdata[j*m+i] = data[i*n+j];
        }
      }
      free (data);
      data = newdata;
    }
    // swap n and m
    I t = m;
    m = n;
    n = t;
    return *this;
  }

  Matrix& Matrix::toZero() {
    memset ( data, D_Zero, m*n*sizeof ( D ) );
    return *this;
  }

  Matrix& Matrix::toId() {
    toZero();
    I smallerdim = m < n ? m : n;
    for ( I i = 0; i < smallerdim; i++ ) {
      VAL ( i, i ) = D_One;
    }
    return *this;
  }

  void Matrix::add ( const Matrix& a, const Matrix& b ) {
    assert ( a.m == b.m && a.n == b.n );
    copy ( a );
    toSum ( b );
  }

  void Matrix::add ( const Matrix& a, const D& summand ) {
    m = a.m;
    n = a.n;
    allocate();
    for ( I i = 0; i < m*n; i++ ) {
      data[i] = a.data[i] + summand;
    }
  }

  Matrix& Matrix::toSum(const Matrix& a) {
    assert(a.m==m && a.n==n);
    for(I i=0; i<m*n; i++){
      data[i]+=a.data[i];
    }
    return *this;
  }

  Matrix& Matrix::toSum(const D& sum) {
    for(I i=0; i<m*n; i++){
      data[i]+=sum;
    }
    return *this;
  }

  void Matrix::sub ( const Matrix& a, const Matrix& b ) {
    assert ( a.m == b.m && a.n == b.n );
    copy ( a );
    toDiff ( b );
  }

  Matrix& Matrix::toDiff(const Matrix& a){
    assert(a.m==m && a.n==n);
    for(I i=0; i<m*n; i++){
      data[i]-=a.data[i];
    }
    return *this;
  }

  void Matrix::mult ( const Matrix& a, const Matrix& b ) {
    assert ( a.n == b.m );
    m = a.m;
    n = b.n;
    I interdim = a.n;
    allocate();
    D d;
    for ( I i = 0; i < m; i++ ) {
      for ( I j = 0; j < n; j++ ) {
        d = 0;
        for ( I k = 0; k < interdim; k++ ) {
          d += a.val ( i, k ) * b.val ( k, j );
        }
        VAL ( i, j ) = d;
      }
    }
  }

  void Matrix::mult ( const Matrix& a, const D& fac ) {
    m = a.m;
    n = a.n;
    allocate();
    for ( I i = 0; i < m*n; i++ ) {
      data[i] = a.data[i] * fac;
    }
  }

  Matrix& Matrix::toMult(const Matrix& a){
    Matrix copy(*this);
    mult(copy,a);
    return *this;
  }

  Matrix& Matrix::toMult ( const D& fac ) {
    for ( I i = 0; i < m*n; i++ ) {
      data[i] *= fac;
    }
    return *this;
  }

  /* special  matrix power
      @see toExp
   */
  void Matrix::exp( const Matrix& a, int exponent ) {
    set(a.m,a.n,a.data);
    toExp ( exponent );
  }

  /* special inplace matrix power:
      @param exp -1 -> inverse; 0 -> Identity Matrix;
      1 -> itself;
      T -> Transpose
  */
  Matrix& Matrix::toExp ( int exponent ) {
    switch ( exponent ) {
      case - 1:
        if ( m == 1 ) VAL ( 0, 0 ) = 1 / VAL ( 0, 0 );
        else if ( m == 2 ) invert2x2();
        else if ( m == 3 ) invert3x3();
        else invertnonzero();
        break;
      case 0:
        toId();
        break;
      case 1: // do nothing
        break;
      case 2: // square
        toMult(*this);
        break;
      case T:
        toTranspose();
        break;
      default:
        if(exponent>0){
          Matrix m(*this);
          for(int i=1; i<exponent; i++){
            toMult(m);
          }
        }else
          assert ( "Exponent < -1" == 0 );
        break;
    }
    return *this;
  }

  bool Matrix::hasNormalEntries() const {
    for(I i=0; i < m*n; i++){
      if(std::isinf(data[i]) || std::isnan(data[i]) ) return false;
    }
    return true;
  }

  Matrix Matrix::secureInverse() const {
    // try first without lambda
    const Matrix& Rinv = (*this)^(-1);
    // if it has NAN or INF entries then regularize
    if(Rinv.hasNormalEntries()){
      return Rinv;
    }else{
      return pseudoInverse(0.000001);
    }
  }


  Matrix Matrix::pseudoInverse(const D& lambda) const {
    Matrix R;
    if(m>n)
      R = this->multTM();
    else
      R = this->multMT();

    // try first without lambda
    Matrix Rinv = R^(-1);
    // if it has NAN or INF entries then regularize
    if(!Rinv.hasNormalEntries()){
      for(I i=0; i < R.getM(); i++){
        R.val(i,i)+= lambda;
      }
      Rinv = R^(-1);
    }
    if(m>n)
      return Rinv* (*this^T);
    else
      return (*this^T)*Rinv;
  }

  Matrix& Matrix::toMap ( D ( *fun ) ( D ) ) {
    I len = m * n;
    for ( I i = 0; i < len; i++ ) {
      data[i] = fun ( data[i] );
    }
    return *this;
  }

  Matrix Matrix::map ( D ( *fun ) ( D ) ) const {
    Matrix result ( *this );
    result.toMap ( fun );
    return result;
  }

  Matrix& Matrix::toMapP ( D param, D ( *fun ) ( D, D ) ) {
    I len = m * n;
    for ( I i = 0; i < len; i++ ) {
      data[i] = fun ( param, data[i] );
    }
    return *this;
  }
  Matrix& Matrix::toMapP ( void* param, D ( *fun ) ( void*, D ) ) {
    I len = m * n;
    for ( I i = 0; i < len; i++ ) {
      data[i] = fun ( param, data[i] );
    }
    return *this;
  }

  Matrix Matrix::mapP ( D param, D ( *fun ) ( D, D ) ) const {
    Matrix result ( *this );
    result.toMapP ( param, fun );
    return result;
  }

  Matrix Matrix::mapP ( void* param, D ( *fun ) ( void*, D ) ) const {
    Matrix result ( *this );
    result.toMapP ( param, fun );
    return result;
  }


  Matrix& Matrix::toMap2 ( D ( *fun ) ( D,D ), const Matrix& b ) {
    assert ( m == b.m && n == b.n );
    I len = m * n;
    for ( I i = 0; i < len; i++ ) {
      data[i] = fun ( data[i], b.data[i] );
    }
    return *this;
  }

  Matrix Matrix::map2 ( D ( *fun ) ( D, D ), const Matrix& a, const Matrix& b ) {
    Matrix result ( a );
    result.toMap2 ( fun,b );
    return result;
  }

  Matrix& Matrix::toMap2P ( D param, D (*fun) (D, D,D ), const Matrix& b ) {
    assert ( m == b.m && n == b.n );
    I len = m * n;
    for ( I i = 0; i < len; i++ ) {
      data[i] = fun ( param, data[i], b.data[i] );
    }
    return *this;
  }

  Matrix& Matrix::toMap2P ( void* param, D ( *fun ) ( void*, D,D ), const Matrix& b ) {
    assert ( m == b.m && n == b.n );
    I len = m * n;
    for ( I i = 0; i < len; i++ ) {
      data[i] = fun ( param, data[i], b.data[i] );
    }
    return *this;
  }

  Matrix Matrix::map2P( D param, D (*fun)(D, D,D), const Matrix& a, const Matrix& b){
    Matrix result (a);
    result.toMap2P(param,fun,b );
    return result;
  }

  Matrix Matrix::map2P ( void* param, D ( *fun ) ( void*, D, D ), const Matrix& a, const Matrix& b ) {
    Matrix result ( a );
    result.toMap2P ( param,fun,b );
    return result;
  }

  Matrix& Matrix::toMultrowwise ( const Matrix& factors ) {
    assert ( m == factors.m && factors.n == 1 );
    for ( I i = 0; i < m; i++ ) {
      for ( I j = 0; j < n; j++ ) {
        VAL ( i, j ) *= factors.val ( i, 0 );
      }
    }
    return *this;
  }

  Matrix& Matrix::toMultcolwise ( const Matrix& factors ) {
    assert ( n == factors.m && factors.n == 1);
    for ( I i = 0; i < m; i++ ) {
      for ( I j = 0; j < n; j++ ) {
        VAL ( i, j ) *= factors.val ( j, 0 );
      }
    }
    return *this;
  }

  Matrix& Matrix::toAbove ( const Matrix& a ) {
    assert ( a.n == this->n);
    data = ( D* ) realloc ( data, sizeof ( D ) * ( this->m * this->n + a.n * a.m ) );
    memcpy ( data + this->m * this->n, a.data, sizeof ( D ) * ( a.n * a.m ) );
    this->m += a.m;
    return *this;
  }

  Matrix& Matrix::toBeside ( const Matrix& a ) {
    assert ( a.m == this->m);

    D* oldData = data;
    I oldN= this->n;
    this->n += a.n;
    buffersize=this->m*this->n;
    data = ( D* ) malloc(sizeof ( D ) * buffersize );
    assert ( data );
    if ( oldData ) { // copy old values
      for ( I i=0;i<this->m * oldN;i++ ) {
          data[ (i/oldN)*this->n + ( i%oldN) ]=oldData[i];
      }
      free ( oldData );
    }
    if ( a.data ) { // copy new values for the new rows
      for ( I i = 0;i < m;i++ ) {
        for ( I j=0;j< a.n;j++ ) {
          val(i,j+oldN) = a.val(i,j);
        }
      }
    }
    return *this;
  }

  int cmpdouble ( const void* a, const void* b ) {
    return * ( ( double* ) a ) < * ( ( double* ) b ) ? -1 : ( * ( ( double* ) a ) > * ( ( double* ) b ) ? 1 : 0 );
  }

  Matrix& Matrix::toSort() {
    qsort ( data, m*n, sizeof ( double ), cmpdouble );
    return *this;
  }

  Matrix& Matrix::reshape ( I _m, I _n) {
    assert(_m*_n <= m*n);
    m = _m;
    n = _n;
    return *this;
  }

  /// adds the given value to the diagonal
  Matrix& Matrix::pluslambdaI ( double lambda ) {
    I smallerdim = std::min ( m, n );
    for ( I i = 0; i < smallerdim; i++ ) {
      VAL ( i, i ) += lambda;
    }
    return *this;
  }


  Matrix Matrix::multrowwise ( const Matrix& factors ) const {
    Matrix result ( *this );
    result.toMultrowwise ( factors );
    return result;
  }

  Matrix Matrix::multcolwise ( const Matrix& factors ) const {
    Matrix result ( *this );
    result.toMultcolwise ( factors );
    return result;
  }

  // multiply Matrix with its transposed: M * M^T
  Matrix Matrix::multMT() const {
    assert ( m != 0 && n != 0 );
    Matrix result ( m, m );
    D d;
    for ( I i = 0; i < m; i++ ) {
      for ( I j = 0; j < m; j++ ) {
        d = 0;
        for ( I k = 0; k < n; k++ ) {
          d += VAL ( i, k ) * VAL ( j, k );
        }
        result.val ( i, j ) = d;
      }
    }
    return result;
  }

  // multiply transpsoed of Matrix with itself: M^T * M
  Matrix Matrix::multTM() const {
    assert ( m != 0 && n != 0 );
    Matrix result ( n, n );
    D d;
    for ( I i = 0; i < n; i++ ) {
      for ( I j = 0; j < n; j++ ) {
        d = 0;
        for ( I k = 0; k < m; k++ ) {
          d += VAL ( k, i ) * VAL ( k, j );
        }
        result.val ( i, j ) = d;
      }
    }
    return result;
  }

  /// returns the product of all elements
  D Matrix::elementProduct() const {
    D rv = 1;
    unsigned int mn = m*n;
    for ( I i = 0; i < mn; i++ ) {
      rv *= data[i];
    }
    return rv;
  }

  /// returns the sum of all elements
  D Matrix::elementSum() const {
    D rv = 0;
    for ( I i = 0; i < m*n; i++ ) {
      rv += data[i];
    }
    return rv;
  }

  /// returns the sum of all elements
  D Matrix::norm_sqr() const {
    // short: map(sqr).elementSum()
    D rv = 0;
    for ( I i = 0; i < m*n; i++ ) {
      rv += data[i]*data[i];
    }
    return rv;
  }

  /// returns a matrix that consists of b below this
  Matrix Matrix::above ( const Matrix& b ) const {
    Matrix r ( *this );
    r.toAbove ( b );
    return r;

  }

  /// returns a matrix that consists of b right beside this
  Matrix Matrix::beside ( const Matrix& b ) const {
    Matrix r ( *this );
    r.toBeside( b );
    return r;

  }

  // adds one or more rows to the existing matrix
  // The data for the new rows is read row-wise.
  Matrix& Matrix::addRows ( I numberRows, const D* _data /*=0*/ ) {
    return toAbove(Matrix(numberRows,n, data));
  }

  Matrix& Matrix::addRows ( I numberRows, const Matrix& dataMatrix ) {
    return toAbove(dataMatrix);
  }


  // adds one or more columns to the existing matrix
  // The data for the new columns is read row-wise.
  Matrix& Matrix::addColumns ( I numberColumns, const D* _data /*=0*/ ) {
    return toBeside(Matrix(m, numberColumns, _data));
  }

  Matrix& Matrix::addColumns ( I numberColumns, const Matrix& dataMatrix ) {
    return toBeside(dataMatrix);
  }

  Matrix& Matrix::removeRows ( I numberRows ) {
    assert ( m>numberRows && "to much rows to remove" );
    return reshape(m-numberRows, n);
  }

  Matrix& Matrix::removeColumns ( I numberColumns ) {
    assert ( n>numberColumns && "to much columns to remove" );
    // internal allocation
    D* oldData = data;
    I newN = n - numberColumns;
    data = ( D* ) malloc ( sizeof ( D ) * m * newN );
    assert ( data );
    if ( oldData ) { // copy old values
      for ( I i=0;i<m;i++ ) {
        for ( I j=0;j< newN;j++ ) {
          data[i*newN+j]=oldData[i*n+j];
        }
      }
      free ( oldData );
    }
    n= newN;
    buffersize=m*n;
    return *this;
  }


  /* inplace matrix invertation:
      Matrix must be SQARE, in addition, all DIAGONAL ELEMENTS MUST BE NONZERO  */
  void Matrix::invertnonzero() {
    assert ( m == n && m > 1 ); // must be of dimension >= 2

    for ( I i = 1; i < m; i++ ) data[i] /= data[0]; // normalize row 0
    for ( I i = 1; i < m; i++ ) {
      for ( I j = i; j < m; j++ ) { // do a column of L
        D sum = 0.0;
        for ( I k = 0; k < i; k++ )
          sum += VAL ( j, k ) * VAL ( k, i );
        VAL ( j, i ) -= sum;
      }
      if ( i == m - 1 ) continue;
      for ( I j = i + 1; j < m; j++ ) { // do a row of U
        D sum = 0.0;
        for ( I k = 0; k < i; k++ )
          sum += VAL ( i, k ) * VAL ( k, j );
        VAL ( i, j ) = ( VAL ( i, j ) - sum ) / VAL ( i, i );
      }
    }
    for ( I i = 0; i < m; i++ ) // invert L
      for ( I j = i; j < m; j++ ) {
        D x = 1.0;
        if ( i != j ) {
          x = 0.0;
          for ( I k = i; k < j; k++ )
            x -= VAL ( j, k ) * VAL ( k, i );
        }
        VAL ( j, i ) = x / VAL ( j, j );
      }
    for ( I i = 0; i < m; i++ )  // invert U
      for ( I j = i; j < m; j++ ) {
        if ( i == j ) continue;
        D sum = 0.0;
        for ( I k = i; k < j; k++ )
          sum += VAL ( k, j ) * ( ( i == k ) ? 1.0 : VAL ( i, k ) );
        VAL ( i, j ) = -sum;
      }
    for ( I i = 0; i < m; i++ )  // final inversion
      for ( I j = 0; j < m; j++ ) {
        D sum = 0.0;
        for ( I k = ( ( i > j ) ? i : j ); k < m; k++ )
          sum += ( ( j == k ) ? 1.0 : VAL ( j, k ) ) * VAL ( k, i );
        VAL ( j, i ) = sum;
      }
  };


  void Matrix::invert2x2() {
    assert ( m == n && m == 2 );
    // high speed version
    D detQ = data[0] * data[3] - data[1] * data[2];
    D tmp = data[0];
    data[0] = data[3] / detQ;
    data[3] = tmp / detQ;
    data[1] /= -detQ;
    data[2] /= -detQ;
  }

  void Matrix::invert3x3() {
    assert ( m == n && m == 3 );
    D Q_adjoint[3][3];
    D detQ = 0;
    //calculate the inverse of Q
    Q_adjoint[0][0] = VAL ( 1, 1 ) * VAL ( 2, 2 ) - VAL ( 1, 2 ) * VAL ( 2, 1 ) ;
    Q_adjoint[0][1] = ( VAL ( 1, 2 ) * VAL ( 2, 0 ) - VAL ( 1, 0 ) * VAL ( 2, 2 ) ) ;
    Q_adjoint[0][2] = VAL ( 1, 0 ) * VAL ( 2, 1 ) - VAL ( 1, 1 ) * VAL ( 2, 0 ) ;
    Q_adjoint[1][0] = ( VAL ( 2, 1 ) * VAL ( 0, 2 ) - VAL ( 0, 1 ) * VAL ( 2, 2 ) ) ;
    Q_adjoint[1][1] = VAL ( 0, 0 ) * VAL ( 2, 2 ) - VAL ( 0, 2 ) * VAL ( 2, 0 ) ;
    Q_adjoint[1][2] = ( VAL ( 0, 1 ) * VAL ( 2, 0 ) - VAL ( 0, 0 ) * VAL ( 2, 1 ) ) ;
    Q_adjoint[2][0] = VAL ( 0, 1 ) * VAL ( 1, 2 ) - VAL ( 1, 1 ) * VAL ( 0, 2 ) ;
    Q_adjoint[2][1] = ( VAL ( 1, 0 ) * VAL ( 0, 2 ) - VAL ( 0, 0 ) * VAL ( 1, 2 ) ) ;
    Q_adjoint[2][2] = VAL ( 0, 0 ) * VAL ( 1, 1 ) - VAL ( 0, 1 ) * VAL ( 1, 0 ) ;
    detQ = VAL ( 0, 0 ) * Q_adjoint[0][0] + VAL ( 0, 1 ) * Q_adjoint[0][1] + VAL ( 0, 2 ) * Q_adjoint[0][2] ;
    for ( I i = 0; i < 3; i++ ) {
      for ( I j = 0; j < 3; j++ ) {
        VAL ( i, j ) = ( Q_adjoint[j][i] ) / detQ  ;
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // normal binary operators

  Matrix Matrix::operator + ( const Matrix& sum ) const {
    Matrix result;
    result.add ( *this, sum );
    return result;
  }

  Matrix Matrix::operator - ( const Matrix& sum ) const {
    Matrix result;
    result.sub ( *this, sum );
    return result;
  }

  /** matrix product*/
  Matrix Matrix::operator * ( const Matrix& fac ) const {
    Matrix result;
    result.mult ( *this, fac );
    return result;
  }
  /** product with scalar (double)*/
  Matrix Matrix::operator * ( const D& scalar ) const {
    Matrix result;
    result.mult ( *this, scalar );
    return result;
  }
  /** special matrix potence:
      @param exp -1 -> inverse; 0 -> Identity Matrix;
      1 -> itself; 2-> Matrix*Matrix^T
      T -> Transpose
  */
  Matrix Matrix::operator ^ ( int exponent ) const {
    Matrix result ( *this );
    result.toExp ( exponent );
    return result;
  }

  /// row-wise multiplication
  Matrix Matrix::operator & (const Matrix& b) const {
    Matrix result ( *this );
    result.toMultrowwise ( b );
    return result;
  }

  bool Matrix::operator == ( const Matrix& c ) const {
    if ( m != c.m || n != c.n ) return false;
    D* p1 = data;
    D* p2 = c.data;
    I len = m * n;
    for ( I i = 0; i < len; i++ ) {
      if ( fabs ( *p1 - *p2 ) > COMPARE_EPS ) {
        return false;
      }
      p1++; p2++;
    }
    return true;
  }

  std::ostream& operator<< ( std::ostream& str, const Matrix& mat ) {
    if ( mat.m == 0 || mat.n == 0 ) return str << "0";
    else {
      //      str << mat.m << "x" << mat.n << " (\n";
      for ( I i = 0; i < mat.m; i++ ) {
        for ( I j = 0; j < mat.n; j++ ) {
          str << mat.val ( i, j ) << " \t";
        }
        if ( i != mat.m - 1 ) str << std::endl;
      }
    }
    //    return str << ")\n";
    return str;
  }

}

#ifdef UNITTEST
#include "matrix.tests.hpp"
#endif
