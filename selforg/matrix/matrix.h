/***************************************************************************
                          matrix.h  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/
// provides Matrix class with convinient operators
//  and fast inversion for nonzero square matrixes
//
/***************************************************************************/

#ifndef MATRIX_H
#define MATRIX_H

#include <string.h>
#include <assert.h>
#include <list>
#include <vector>
#include <cstdlib>

#include <iostream>

#include "storeable.h"

// TODO: add doxygen section

/**
 * namespace for the matrix library
 *@author Georg Martius
 */
namespace matrix{

  /// integer constant for use with exp function and (^) operator to transpose the matrix
  extern const int T;

/// type for matrix indices
  typedef unsigned int I;
  /// type for matrix elements
  typedef double D;

  class Matrix;
  typedef std::vector<Matrix> Matrices;

#define D_Zero 0
#define D_One 1
  /** Matrix type. Type D is datatype of matrix elements,
   * which is fixed to double.
   * Type I is the indextype of matrix elements,
   * which is fixed to unsigned int.
   * There are basicly two different types of operation:
   * Inplace operations and copy operations.
   * Please use the latter ones unless you know what you are doing.
   * Just in case of critical performance optimisation use the inplace
   * operations.
   * The most convinient way is to use the overloaded operators
   * (like + * ...).
   * All constructed matrices are initialized with zero elements
   * (unless data is given).
   * All functions perform range checks if in debug mode
   * (i.e. if NDEBUG is not defined).
   * Please use debug version (default) for testing
   * @see examples/matrix/matrixexample.cpp
   *
   * @author Georg Martius
   */
  class Matrix : public Storeable {
  public:
    /// default constructor: zero matrix (0x0)
    Matrix()
      : m(0), n(0), buffersize(0), data(0) {};
    /** constucts a matrix with the given size.
        If _data is null then the matrix is filled with zeros.
        otherwise matrix will be filled with _data in a row-wise manner.
        In this case _data must be at least _m*_n elements long
    */
    Matrix(I _m, I _n, const D* _data=0);
    /** constucts a matrix with the given size and fills it with the default value
    */
    Matrix(I _m, I _n, D def);
    /// constucts a instance on the base of a deep copy of the given matrix
    Matrix (const Matrix& c);
    /// copy move constructor
    Matrix (Matrix&& c);
    ~Matrix() { if(data) free(data); };

  public:
    //  /////////////////// Accessors ///////////////////////////////
    /** @return number of rows */
    I getM() const { return m; };
    /** @return number of columns */
    I getN() const { return n; };
    /// @return number number elements in the matrix (getM()*getN())
    I size() const { return n*m; };


    /** @return element at position i,j (row, column index) */
    inline D val(I i, I j) const {
      assert( i<m && j<n);
      return data[i*n+j];
    };
    /** @return reference to element at position i,j
        (can be used as left side value) */
    inline D& val(I i, I j) {
      assert( i<m && j<n);
      return data[i*n+j];
    };

    /** @return element at position i,j (row, column index) and 0 if out of bounds */
    inline D valDef0(I i, I j) const {
      if(i<m && j<n)
        return data[i*n+j];
      else return 0;
    };

    /** sets the size of the matrix and maybe the data if given (row-wise).
        If data=null then the matrix is set to zero
        @see toZero()
        @see constructor Matrix(m,n,data)
    */
    void set(I _m, I _n, const D* _data=0);
    /** sets the data (row-wise).
        @param _data if null then matrix elements are set to zero
        otherwise the field MUST have the length should be getM()*getN()*/
    void set(const D* _data);
    /** @return row-vector(as 1xN matrix) containing the index'th row */
    Matrix row(I index) const;
    /** @returns submatrix (as KxN matrix)
        containing row from startindex to endindex inclusively (K=stopindex-endindex)
        indices can be out of bounds, they are clipped in any case
    */
    Matrix rows(I startindex, I endindex) const;
    /** @returns column-vector(as Mx1 matrix) containing the index'th column */
    Matrix column(I index) const;
    /** @returns submatrix (as MxK matrix)
        containing column from startindex to endindex inclusively (K=endindex-startindex)
        indices can be out of bounds, they are clipped in any case
    */
    Matrix columns(I startindex, I endindex) const;


    /** stores the content of the matrix (row-wise) in the given buffer
        @param buffer Buffer for storing the elements (should have the length given by len)
        @param len Length of the provided buffer.
               In any case only min(len, getM()*getN()) elements are copied.
        @return number of actually written elements
     */
    int convertToBuffer(D* buffer, I len) const;

    /** @return a list of the content of the matrix (row-wise)
     */
    std::list<D> convertToList() const;

    /// returns a pointer to the data. UNSAFE!!!
    const D* unsafeGetData() const{return data;}

    /*       STOREABLE       */
    /** stores the Matrix into the given file stream (same as write)
     */
    bool store(FILE* f) const;

    /** reads a Matrix from the given file stream
        uses read (or old binary format)
     */
    bool restore(FILE* f);

    /** writes the Matrix into the given file stream (ascii)
     */
    bool write(FILE* f) const;

    /** reads a Matrix from the given file stream (ascii)
     */
    bool read(FILE* f, bool skipIdentifier = false);


  public:
    // ////////////////////////////////////////////////////////////////////
    // /////////////  operations  /////////////////////////////
    //  (the result of the operation is not stored in one of the operands)
    void add(const Matrix& a, const Matrix& b); ///< addition:    this = a + b
    void add(const Matrix& a, const D& summand); /// add scalar to each element
    void sub(const Matrix& a, const Matrix& b); ///< subtraction: this = a - b
    void mult(const Matrix& a, const Matrix& b);///< multiplication: this = a * b
    void mult(const Matrix& a, const D& fac);///< scaling: this = a * fac

    void exp(const Matrix& a, int exponent);///< exponent, this = a^i, @see toExp

    /// returns true if matrix is a 0x0 matrix
    bool isNulltimesNull() const;

    /// returns true if matrix is a vector
    bool isVector() const;

    /** bytewise comparison (compares data buffer bytewise, which implies that
        n1*m1 == n2*m2 but not necessarily n1==n2) */
    bool equals(const Matrix& a) const;

    /// @return true of a and this have the same size
    bool hasSameSizeAs(const Matrix& a) const { return n==a.getN() && m==a.getM(); };

    /** calculates the pseudoinverse, depending on the shape of the matrix
        the left or right pseudoinverse is used.
        If the matrix has more columns than rows then we use
        \f[A^{+} = (A^T A + \lambda \mathbb I)^{-1}A^T\f]
        otherwise
        \f[A^{+} = A^T(A A^T + \lambda \mathbb I)^{-1}\f]
     */
    Matrix pseudoInverse(const D& lambda = 1e-8) const ;

    /** calculates the secure inverse of a square matrix.
        If singular then the pseudoinverse is used.
     */
    Matrix secureInverse() const;

    /** returns true if all entries are normal floating point numbers,
        otherwise false (e.g. for nan and inf)*/
    bool hasNormalEntries() const;


    /**  maps the matrix to a new matrix
         with all elements mapped with the given function
    */
    Matrix map(D (*fun)(D)) const;
    /**  like map but with additional double parameter for the mapping function
         (first argument of fun is parameter, the second is the value)*/
    Matrix mapP(D param, D (*fun)(D, D)) const;
    /**  like map but with additional arbitrary parameter for the mapping function */
    Matrix mapP(void* param, D (*fun)(void*, D)) const;

    // Exotic operations ///////////
    /** binary map operator for matrices.
       The resulting matrix consists of the function values applied to the elements of a and b.
       In haskell this would something like: map (uncurry . fun) $ zip a b
    */
    static Matrix map2( D (*fun)(D,D), const Matrix& a, const Matrix& b);

    /** like map2 but with additional parameter.
        The first argument of fun is the parameter and the second and third
        comes from the matrix elements.
        In haskell this would something like: map (uncurry . (fun p)) $ zip a b
     */
    static Matrix map2P( D param, D (*fun)(D, D,D), const Matrix& a, const Matrix& b);
    /** like map2P but with arbitrary paramters (void*) instead of double
     */
    static Matrix map2P( void* param, D (*fun)(void*, D,D), const Matrix& a, const Matrix& b);



    /** row-wise multiplication
        @param factors column vector (Mx1) of factors, one for each row
    */
    Matrix multrowwise(const Matrix& factors) const;
    /** column-wise multiplication
        @param factors column vector (Mx1) of factors, one for each column
    */
    Matrix multcolwise(const Matrix& factors) const;

    /// optimised multiplication of Matrix with its transposed: M * M^T
    Matrix multMT() const;
    /// optimised multiplication of transpsoed of Matrix with itself: M^T * M
    Matrix multTM() const;

    /// returns the product of all elements (\f$ \Pi_{ij} m_{ij} \f$)
    D elementProduct() const;
    /// returns the sum of all elements (\f$ \sum_{ij} m_{ij} \f$)
    D elementSum() const;

    /** returns the sum of all squares of all elements (\f$ \sum_{ij} m_{ij}^2 \f$)
        this is also known as the square of the Frobenius norm.
     */
    D norm_sqr() const;

    /// returns a matrix that consists of this matrix above A (number of rows is getM + a.getM())
    Matrix above(const Matrix& a) const ;
    /// returns a matrix that consists of this left beside A  (number of columns is getN + a.getN())
    Matrix beside(const Matrix& a) const ;

  public:   // normal binary Operators
    /// deep copy
    Matrix& operator = (const Matrix& c) { copy(c); return *this; }
    /// deep copy move operator
    Matrix& operator = (Matrix&&c);
    /// sum of two matrices
    Matrix operator +  (const Matrix& sum) const;
    //    Matrix operator +  (const D& sum) const; /// new operator (guettler)
    /// difference of two matrices
    Matrix operator -  (const Matrix& sum) const;
    /** matrix product*/
    Matrix operator *  (const Matrix& fac) const;
    /** product with scalar (D) (only right side) */
    Matrix operator *  (const D& fac) const;
    /** special matrix potence:
        @param exponent -1 -> inverse;
                       0 -> Identity Matrix;
                    1 -> itself;
                    T -> Transpose
    */
    Matrix operator ^ (int exponent) const;
    /// row-wise multiplication
    Matrix operator & (const Matrix& b) const;
    /// combined assigment operator (higher performance)
    Matrix& operator += (const Matrix& c) {toSum(c);   return *this; }
    /// combined assigment operator (higher performance)
    Matrix& operator -= (const Matrix& c) {toDiff(c);  return *this; }
    /// combined assigment operator (higher performance)
    Matrix& operator *= (const Matrix& c) {
      Matrix result;
      result.mult(*this, c);
      this->copy(result);
      return *this;
    }
    /// combined assigment operator (higher performance)
    Matrix& operator *= (const D& fac) {toMult(fac); return *this; }
    /// combined assigment operator (higher performance)
    Matrix& operator &= (const Matrix& c) {toMultrowwise(c); return *this; }

    /// comparison operator (compares elements with tolerance distance of COMPARE_EPS)
    bool operator == (const Matrix& c) const;
    /** printing operator:
        output format: mxn (\n row0\n..rown \n) where rowX is tab seperated list of values
    */
    friend std::ostream& operator<<(std::ostream& , const Matrix&);

  public:
    // /////////////////// inplace Operators ///////////////////////////////
    /** performs a deep copy of the given matrix */
    void copy(const Matrix& c){ // Deep copy
      m=c.m; n=c.n;
      allocate();
      memcpy(data,c.data,m*n*sizeof(D));
    }

    Matrix& toTranspose();  ///< inplace transpose
    Matrix& toZero();       ///< inplace converts matrix to zero matrix
    Matrix& toId();         ///< inplace converts matrix to identity (use ^0 to get a copy version of it)
    /// inplace addition: this = this + a
    Matrix& toSum(const Matrix& a);
    /// inplace addition: this = this + a, where a is a scalar
    Matrix& toSum(const D& sum);

    /// inplace subtraction: this = this - a
    Matrix& toDiff(const Matrix& a);

    /// inplace multiplication: this = this * a
    Matrix& toMult(const Matrix& a);
    /// inplace multiplication with scalar: this = this*fac
    Matrix& toMult(const D& fac);

    /** special inplace matrix power:
        @param exponent -1 -> inverse; (matrix MUST be SQUARE and NONZERO)
                    0 -> Identity Matrix;
                    1 -> itself;
                    n -> n-th power;
                    T -> Transpose
    */
    Matrix& toExp(int exponent);
    /**  inplace mapping of matrix elements (element-wise application) */
    Matrix& toMap(D (*fun)(D));
    /**  like toMap, but with an extra double parameter for the mapping function. */
    Matrix& toMapP(D param, D (*fun)(D, D));
    /**  like toMap, but with an extra arbitrary parameter for the mapping function. */
    Matrix& toMapP(void* param, D (*fun)(void*, D));

    /**  like toMap, but with using 2 matrices */
    Matrix& toMap2(D (*fun)(D,D), const Matrix& b);

    /**  like toMap2, but with additional parameter */
    Matrix& toMap2P( D param, D (*fun)(D, D,D), const Matrix& b);
    /**  like toMap2P, but with arbitrary parameter */
    Matrix& toMap2P( void* param, D (*fun)(void*, D,D), const Matrix& b);

    // Exotic operations
    /** Inplace row-wise multiplication
        @param factors column vector of factors, one for each row
    */
    Matrix& toMultrowwise(const Matrix& factors);
    /** Inplace column-wise multiplication
        @param factors column vector of factors, one for each column
    */
    Matrix& toMultcolwise(const Matrix& factors);

    /// sets the matrix a below this matrix
    Matrix& toAbove(const Matrix& a);
    /// sets the matrix a right beside this matrix
    Matrix& toBeside(const Matrix& a);

    /// sorts the matrix (rowwise)
    Matrix& toSort();

    /** reshapes the matrix without destroying the data.
        Remember: The data is stored rowwise.

        Only shrinking is allowed i.e. m*n must be lower or equal to getM()*getN()
    */
    Matrix& reshape(I m, I n);

    /// adds the given value to the diagonal
    Matrix& pluslambdaI(double lambda = 1e-8);

    /** adds one or more rows to the existing matrix and fills it with the given data
     *
     * same as toAbove(Matrix(numberRows,getN(),data))
     * @param numberRows number of rows to add (this extends m)
     * @param _data data to add
     * @return the address of the matrix itself
     */
    Matrix& addRows(I numberRows, const D* _data=0);

    /**
     * same as toAbove(dataMatrix)
     * @param numberRows number of rows to add (unused)
     * @param dataMatrix matrix which contains the data of the new rows
     * @return the address of the matrix itself
     */
    Matrix& addRows(I numberRows, const Matrix& dataMatrix);

    /** adds one or more columns to the existing matrix
     * same as toBeside(Matrix(getM, numberColumns, _data))
     * @see toBeside()
     * @param numberColumns number of columns to add (this extends n)
     * @param _data data to add
     * @return the address of the matrix itself
     */
     Matrix& addColumns(I numberColumns, const D* _data=0);

    /**
     * same as toBeside(dataMatrix)
     * @see toBeside()
     * @param numberColumns number of columns to add (unused)
     * @param dataMatrix matrix which contains the data of the new rows
     * @return the address of the matrix itself
     */
    Matrix& addColumns(I numberColumns, const Matrix& dataMatrix);


    /** removes one or more rows from the end if an existing matrix (inplace!),
        same as reshape(getM()-numberRows, getN());
     * @param numberRows number of rows to remove (from the end) (this reduces m)
     * @return the address of the matrix itself
     */
    Matrix& removeRows(I numberRows);

    /** removes one or more columns from the end of the existing matrix (inplace!)
     * resets the size of the matrix and deletes the appropiate data.
     * @param numberColumns number of columns to remove (this reduces n)
     * @return the address of the matrix itself
     */
    Matrix& removeColumns(I numberColumns);

  private:
    // NOTE: buffersize determines available memory storage.
    // m and n define the actual size
    I m, n;
    I buffersize;  // max number if elements
    D* data;      // where the data contents of the matrix are stored


  private: // internals
    void allocate();  //internal allocation
    /*inplace matrix invertation:
        Matrix must be SQARE, in addition, all DIAGONAL ELEMENTS MUST BE NONZERO
        (positive definite)
    */

    void invertnonzero();
    void invert3x3();
    void invert2x2();
  };

} // namespace matrix
#endif
