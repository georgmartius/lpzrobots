/***************************************************************************
                          matrix.h  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/
// provides Matrix class with convinient operators 
//  and fast inversion for nonzero square matrixes
//
// $Log$
// Revision 1.2  2006-07-14 12:24:01  martius
// selforg becomes HEAD
//
// Revision 1.1.2.2  2006/07/14 08:56:53  der
// New function NullTimesNull
//
// Revision 1.1.2.1  2006/07/10 12:01:02  martius
// Matrixlib moved to selforg
//
// Revision 1.20.6.4  2006/03/30 23:08:53  martius
// comments
//
// Revision 1.20.6.3  2006/03/30 12:34:45  martius
// documentation updated
//
// Revision 1.20.6.2  2006/03/29 15:12:46  martius
// column accessor function added
//
// Revision 1.20.6.1  2005/12/06 17:38:10  martius
// *** empty log message ***
//
// Revision 1.20  2005/10/21 11:58:25  martius
// map2 (similar to map but for 2 matrices)
// changed naming of functions to be more consistent.
//  Functions with "to" in front indicate the change of this. (Still not consistent with add, mult ...)
//
// Revision 1.19  2005/10/06 17:10:06  martius
// convertToList
// above and toAbove
//
// Revision 1.18  2005/09/21 08:43:01  martius
// convertToBuffer is const
//
// Revision 1.17  2005/08/06 20:47:36  martius
// Commented
//
// Revision 1.16  2005/07/21 15:13:36  martius
// mapP addid (mapping with additional parameter)
//
// Revision 1.15  2005/06/21 15:35:21  martius
// hide invert3x3 and invert_nonzero for AVR to minimize binary size
//
// Revision 1.14  2005/06/17 15:19:03  martius
// version workes with avr
//
// Revision 1.13  2005/06/10 08:23:15  martius
// mult???wise are copy operations now!
// toMult???wise are the inplace variant.
//
// Revision 1.11  2005/06/09 11:52:03  martius
// multMT (M * M^T) and multTM (M^T * M)
//
// Revision 1.10  2005/06/02 22:48:54  martius
// copy is inline and works correct now
//
// Revision 1.9  2005/06/02 08:48:31  martius
// mult_row/column_wise
// convertToBuffer
//
// Revision 1.8  2005/05/30 22:40:56  martius
// map becomes toMap and the new map returns a new matrix
// exp becomes toExp
//
// Revision 1.7  2005/05/30 21:46:54  martius
// *** empty log message ***
//
// Revision 1.6  2005/05/30 17:21:26  martius
// added zero
// set() and constructor(m,n,0) initialise with zero
// id returns void (more consistent)
//
// Revision 1.5  2005/05/30 16:42:56  martius
// map function included (component-wise function application)
//
// Revision 1.4  2005/05/30 11:28:09  martius
// marked "row" as untested
//
// Revision 1.3  2005/05/30 10:14:54  martius
// 3x3 explicit matrix inversion
//
// Revision 1.2  2005/05/30 09:45:46  martius
// Working. Interface not tested in pratice
//
// Revision 1.1  2005/05/29 22:43:08  martius
// proper Makefile with dependencies and tag generator
//
/***************************************************************************/

#ifndef MATRIX_H
#define MATRIX_H

#include <string.h>
#include <assert.h>
#include <list>

#ifndef AVR
#include <iostream>
#endif
using namespace std;

// TODO: add doxygen section

/**
 * namespace for matrix library
 *@author Georg Martius
 */
namespace matrix{

  /// integer constant for use with exp function and (^) operator to transpose the matrix 
  extern const int T;

  /// type for matrix elements
  typedef double D;
#define D_Zero 0
#define D_One 1
  /** Matrix type. Type D is datatype of matrix elements, which is fixed to double.
      There are basicly two different types of operation:
      Inplace operations and copy operations. 
      Please use the latter ones unless you know what you are doing. 
      Just in case of critical performance optimisation use the inplace operations.
      The most convinient way is to use the overloaded operators (like + * ...).
      All constructed matrices are initialised with zero elements (unless data is given).
      All functions perform range checks if in debug mode (NDEBUG is not defined). 
      Please use -lmatrix_debug for testing.
      @author Georg Martius
  */
  class Matrix{  
  public:
    /// default constructor: zero matrix (0x0)
    Matrix() 
      : m(0), n(0), buffersize(0), data(0) {};      
    /** constucts a matrix with the given size. 
	If _data is null then the matrix is filled with zeros.
	otherwise matrix will be filled with _data in a row-wise manner.
	In this case _data must be at least _m*_n elements long
    */ 
    Matrix(unsigned short _m, unsigned short _n, const D* _data=0);
    /// constucts a instance on the base of a deep copy of the given matrix
    Matrix (const Matrix& c);	
    ~Matrix() { if(data) delete[] data; };

  public: 
    //  /////////////////// Accessors ///////////////////////////////
    /** @return number of rows */
    unsigned short getM() const { return m; };
    /** @return number of columns */
    unsigned short getN() const { return n; };
    /** @return element at position i,j (row, column index) */
    D val(unsigned short i, unsigned short j) const { 	
      assert( i<m && j<n);	 
      return data[i*n+j];
    };
    /** @return reference to element at position i,j 
	(can be used as left side value) */
    D& val(unsigned short i, unsigned short j) { 	
      assert( i<m && j<n);
      return data[i*n+j];
    };
    /** sets the size of the matrix and maybe the data if given (row-wise).
        If data=null then the matrix is set to zero 
	@see toZero()
        @see constructor Matrix(m,n,data)
    */
    void set(unsigned short _m, unsigned short _n, const D* _data=0);
    /** sets the data (row-wise).
        @param _data if null then matrix elements are set to zero
        otherwise the field MUST have the length should be getM()*getN()*/
    void set(const D* _data);
    /** @return row-vector(as 1xN matrix) containing the index'th row */
    Matrix row(unsigned short index) const;
    /** @returns column-vector(as Nx1 matrix) containing the index'th column */
    Matrix column(unsigned short index) const;

    /** stores the content of the matrix (row-wise) in the given buffer
	@param buffer Buffer for storing the elements (should have the length given by len)
	@param len Length of the provided buffer.
               In any case only min(len, getM()*getN()) elements are copied.
	@return number of actually written elements
     */
    int convertToBuffer(D* buffer, unsigned int len) const;
    /** @return a list of the content of the matrix (row-wise)
     */
    list<D> convertToList() const;

  public:
    // ////////////////////////////////////////////////////////////////////
    // /////////////  operations  /////////////////////////////
    //  (the result of the operation is not stored in one of the operands)
    void add(const Matrix& a, const Matrix& b); ///< addition:    this = a + b
    void sub(const Matrix& a, const Matrix& b); ///< subtraction: this = a - b
    void mult(const Matrix& a, const Matrix& b);///< multiplication: this = a * b
    void mult(const Matrix& a, const D& fac);///< scaling: this = a * fac

    /// returns true if matrix is a 0x0 matrix
    bool isNulltimesNull();

    /**  maps the matrix to a new matrix 
	 with all elements mapped with the given function 
    */
    Matrix map(D (*fun)(D)) const;
    /**  like map but with additional parameter for the mapping function
    */
    Matrix mapP(void* param, D (*fun)(void*, D)) const;

    // Exotic operations ///////////
    /** binary map operator for matrices. 
       The resulting matrix consists of the function values applied to the elements of a and b.
       In haskell this would something like: map (uncurry . fun) $ zip a b
    */
    static Matrix map2( D (*fun)(D,D), const Matrix& a, const Matrix& b) {
      assert(a.m == b.m && a.n == b.n);
      Matrix result(a);
      unsigned int len = a.m*a.n;
      for(unsigned short i=0; i < len; i++){
	result.data[i] = fun(a.data[i], b.data[i]);
      }    
      return result;
    }

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

    /// returns the product of all elements (\Pi_{ij} m_{ij})
    D elementProduct() const;
    /// returns the sum of all elements (\Sum_{ij} m_{ij})
    D elementSum() const;
    
    /// returns a matrix that consists of this above A
    Matrix above(const Matrix& a) const ;
          
  public:   /// normal binary Operators      
    /// deep copy
    Matrix& operator = (const Matrix& c) { copy(c); return *this; }
    Matrix operator +  (const Matrix& sum) const;
    Matrix operator -  (const Matrix& sum) const;
    /** matrix product*/
    Matrix operator *  (const Matrix& fac) const;
    /** product with scalar (D) */
    Matrix operator *  (const D& fac) const;
    /** special matrix potence: 
	@param exponent -1 -> inverse;
	               0 -> Identity Matrix; 
	            1 -> itself;
	            T -> Transpose
    */
    Matrix operator ^ (int exponent) const;
    /// performant combined assigment operators
    Matrix& operator += (const Matrix& c) {toSum(c);   return *this; }
    Matrix& operator -= (const Matrix& c) {toDiff(c);  return *this; }
    Matrix& operator *= (const Matrix& c) {
      Matrix result;
      result.mult(*this, c);
      return *this; 
    }
    Matrix& operator *= (const D& fac) {toMult(fac); return *this; }

#ifndef AVR
    /// comparison operator (compares elements with tolerance distance of COMPARE_EPS)
    bool operator == (const Matrix& c) const;
    /** printing operator:
        output format: mxn (\n row0\n..rown \n) where rowX is tab seperated list of values
    */
    friend ostream& operator<<(ostream& , const Matrix&);
#endif

  public:
    // /////////////////// inplace Operators ///////////////////////////////
    /** performs a deep copy of the given matrix */
    void copy(const Matrix& c){ // Deep copy
      m=c.m; n=c.n;
      allocate();
      memcpy(data,c.data,m*n*sizeof(D));    
    }

    void toTranspose();  ///< inplace transpose
    void toZero();       ///< inplace converts matrix to zero matrix
    void toId();         ///< inplace converts matrix to identity (use ^0 to get a copy version of it)
    /// inplace addition: this = this + a
    void toSum(const Matrix& a) {
      assert(a.m==m && a.n==n);
      for(unsigned short i=0; i<m*n; i++){
	data[i]+=a.data[i];
      }
    }
    /// inplace subtraction: this = this - a
    void toDiff(const Matrix& a){
      assert(a.m==m && a.n==n);
      for(unsigned short i=0; i<m*n; i++){
	data[i]-=a.data[i];
      }
    }

    /// inplace multiplication with scalar: this = this*fac
    void toMult(const D& fac);     

    /** special inplace matrix potence: 
	@param exponent -1 -> inverse; (matrix MUST be SQUARE and NONZERO)
                    0 -> Identity Matrix; 
	            1 -> itself;
	            T -> Transpose
    */
    void toExp(int exponent);
    /**  inplace mapping of matrix elements (element-wise application) */
    void toMap(D (*fun)(D));
    /**  like toMap, but with an extra parameter for the mapping function. */
    void toMapP(void* param, D (*fun)(void*, D));
    // Exotic operations
    /** Inplace row-wise multiplication
	@param factors column vector of factors, one for each row 
    */
    void toMultrowwise(const Matrix& factors);
    /** Inplace column-wise multiplication
	@param factors column vector of factors, one for each column
    */
    void toMultcolwise(const Matrix& factors);    

    /// sets the matrix a below (this) matrix
    void toAbove(const Matrix& a);

            
  private:
    // NOTE: buffersize determines available memory storage. 
    // m and n define the actual size
    unsigned short m, n;
    unsigned int buffersize;  // max number if elements
    D* data;      // where the data contents of the matrix are stored    


  private: // internals
    void allocate();  //internal allocation 
    /*inplace matrix invertation:
	Matrix must be SQARE, in addition, all DIAGONAL ELEMENTS MUST BE NONZERO
	(positive definite)
    */
#ifndef AVR
    void invertnonzero();
    void invert3x3();
#endif
    void invert2x2();
  };

} // namespace matrix
#endif
