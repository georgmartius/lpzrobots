/***************************************************************************
                          matrix.tests.cpp  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/
// 
// $Log$
// Revision 1.3  2006-07-20 17:14:36  martius
// removed std namespace from matrix.h
// storable interface
// abstract model and invertablemodel as superclasses for networks
//
// Revision 1.2  2006/07/14 12:24:01  martius
// selforg becomes HEAD
//
// Revision 1.1.2.1  2006/07/10 12:01:02  martius
// Matrixlib moved to selforg
//
// Revision 1.15  2005/10/21 11:58:25  martius
// map2 (similar to map but for 2 matrices)
// changed naming of functions to be more consistent.
//  Functions with "to" in front indicate the change of this. (Still not consistent with add, mult ...)
//
// Revision 1.14  2005/10/06 17:10:06  martius
// convertToList
// above and toAbove
//
// Revision 1.13  2005/08/06 20:47:36  martius
// Commented
//
// Revision 1.12  2005/06/17 15:18:09  martius
// 2x2 Inversion tested
//
// Revision 1.11  2005/06/10 08:21:50  martius
// mult???wise are copy operations now!
// toMult???wise are inplace instead
//
// Revision 1.10  2005/06/09 11:52:03  martius
// multMT (M * M^T) and multTM (M^T * M)
//
// Revision 1.9  2005/06/02 22:49:33  martius
// speed tests extended
//
// Revision 1.8  2005/06/02 08:49:26  martius
// mult_row/column_wise
//
// Revision 1.7  2005/05/30 22:40:56  martius
// map becomes toMap and the new map returns a new matrix
// exp becomes toExp
//
// Revision 1.6  2005/05/30 21:46:54  martius
// *** empty log message ***
//
// Revision 1.5  2005/05/30 16:43:02  martius
// map function included (component-wise function application)
//
// Revision 1.4  2005/05/30 10:15:47  martius
// proper log entry in header
//
/***************************************************************************/

// //////////////////////////////////////////////////////////////////////////////
// ////////////////// UNIT TESTS for matrix lib /////////////////////////////////////////////////

#ifdef UNITTEST
#include "unit_test.hpp"
using namespace matrix;
using namespace std;

const D EPS=1e-9;
bool comparetoidentity(const Matrix& m)  {
  int worstdiagonal = 0;
  D maxunitydeviation = 0.0;
  D currentunitydeviation;
  for ( int i = 0; i < m.getM(); i++ )  {
    currentunitydeviation = m.val(i,i) - 1.;
    if ( currentunitydeviation < 0.0) currentunitydeviation *= -1.;
    if ( currentunitydeviation > maxunitydeviation )  {
      maxunitydeviation = currentunitydeviation;
      worstdiagonal = i;
    }
  }
  int worstoffdiagonalrow = 0;
  int worstoffdiagonalcolumn = 0;
  D maxzerodeviation = 0.0;
  D currentzerodeviation ;
  for ( int i = 0; i < m.getM(); i++ )  {
    for ( int j = 0; j < m.getN(); j++ )  {
      if ( i == j ) continue;  // we look only at non-diagonal terms
      currentzerodeviation = m.val(i,j);
      if ( currentzerodeviation < 0.0) currentzerodeviation *= -1.0;
      if ( currentzerodeviation > maxzerodeviation )  {
	maxzerodeviation = currentzerodeviation;
	worstoffdiagonalrow = i;
	worstoffdiagonalcolumn = j;
      }

    }
  }
//   cout << "\tWorst diagonal value deviation from unity: " 
//        << maxunitydeviation << " at row/column " << worstdiagonal << endl;
//   cout << "\tWorst off-diagonal value deviation from zero: " 
//        << maxzerodeviation << " at row = " << worstoffdiagonalrow 
//        << ", column = " << worstoffdiagonalcolumn << endl;
  return (maxunitydeviation < EPS && maxzerodeviation < EPS); 
}


UNIT_TEST_DEFINES

DEFINE_TEST( check_creation ) {  
  cout << "\n -[ Creation and Access ]-\n";
  Matrix M1(3,3);  
  D testdata[9]={1,0,0, 0,1,0, 0,0,1};
  Matrix M2(3,3);
  M2.set(testdata);
  M1.toId();
  unit_assert( "id=identity_set", M1 == M2 );
  D testdata2[12]={1,2,3, 4,5,6, 7,8,9, 10,11,12};
  Matrix M3(4,3, testdata2);
  unit_assert( "dimension of matrix", M3.getM() == 4 &&  M3.getN() == 3 );
  unit_assert( "field check", 
	       M3.val(0,2) == 3 &&  M3.val(2,1) == 8 &&  M3.val(3,0) == 10 );
  Matrix M4(M3);  
  unit_assert( "copy constructor", M3 == M4 );
  Matrix M5(1,3,testdata2+3);  
  unit_assert( "row", M3.row(1) == M5 );

  unit_pass();
}

DEFINE_TEST( check_vector_operation ) {  
  cout << "\n -[ Vector Operations ]-\n";
  D testdata[3]={-1,3,2};
  const Matrix V1(1,3, testdata);
  const Matrix V2(3,1, testdata);  
  Matrix V3(V1);  

  V3.toTranspose();
  unit_assert( "transpose", V3 == V2 );
  V3.toTranspose();
  unit_assert( "double transpose", V3 == V1 );
  D testdata2[3]={-2,6,4};
  Matrix V4(1,3,testdata2);    
  V3.add(V1,V1);
  unit_assert( "add", V3 == V4 );
  D testdata3[3]={1,-3,-2};
  Matrix V5(1,3,testdata3);
  V4.sub(V1,V3);
  unit_assert( "sub", V4 == V5 );

  D testdata4[3]={3,-9,-6};
  V5.set(testdata4);
  V4.copy(V1);
  V4.toMult(-3.0);
  unit_assert( "mult with scalar I", V4 == V5 );
  V4.mult(V1,-3.0);
  unit_assert( "mult with scalar II", V4 == V5 );

  double f = 14;
  Matrix V6(1,1,&f);
  V3.copy(V1);  
  V3.toTranspose();
  V4.mult(V1,V3);
  unit_assert( "scalarproduct", V4 == V6 );
  V4.copy(V1);
  unit_assert( "scalarproduct with exp(2)", V4.multMT() == V6 );
  V4.mult(V3,V1);
  unit_assert( "vector^T*vector=matrix", V4.getM() == 3);
  unit_assert( "vector^T*vector=exp(2)", V3.multMT().getM() == 3);
    
  unit_pass();
}

DEFINE_TEST( check_matrix_operation ) {  
  cout << "\n -[ Matrix Operations ]-\n";
  D testdata[6]={1,2,3, 4,5,6 };
  const Matrix M1(2,3, testdata);
  D testdata2[6]={1,4, 2,5, 3,6 };
  const Matrix M2(3,2, testdata2);  
  Matrix M3(M1);  

  M3.toTranspose();
  unit_assert( "transpose", M3 == M2 );
  D testdata3[6]={2,4,6, 8,10,12};
  Matrix M4(2,3,testdata3);    
  M3.add(M1,M1);
  unit_assert( "add", M3 == M4 );

  D testdata4[6]={-0.5, -1, -1.5,  -2, -2.5, -3};
  M3.set(testdata4);
  M4.copy(M1);
  M4.toMult(-0.5);
  unit_assert( "mult with scalar", M4 == M3 );

  D testdata5[6] = {2, -1,  0, 3,  2, -2};
  D testdata6[4] = {8, -1,  20, -1};
  Matrix M5 (3,2, testdata5);
  Matrix M6(2,2,testdata6);
  M4.mult(M1,M5);
  unit_assert( "mult(matrix, matrix)", M4 == M6 );

  M3.copy(M1);  
  M4.copy(M1);  
  M4.toExp(1);
  unit_assert( "exp(1)", M3 == M4 );
  M3.toTranspose();
  M4.toExp(T);
  unit_assert( "exp(T)=transpose", M3 == M4 );
  M3.toId();
  M4.toExp(0);
  unit_assert( "exp(0)=id", M3 == M4 );

  D testdata7[16] = {1,2,3,4, -4,2,1,3, 0.3,-0.9, 4, -3, 1,0.5,0.3,5.0};
  Matrix M7(4,4,testdata7);
  Matrix M8(M7);  
  M7.toExp(-1);  
  M4.mult(M7,M8);
  unit_assert( "exp(-1)*exp(1)=id",   comparetoidentity(M4) );

  D testdata9[6] = {sin(1.0),sin(2.0),sin(3.0), sin(4.0),sin(5.0),sin(6.0) };
  Matrix M9(2,3,testdata9);  
  M4.copy(M1);
  M4.toMap(sin);
  unit_assert( "map(sin)",   M4 == M9 );

  D testdata10[6] = {2,4,6, -0.4,-0.5,-0.6 };
  D testdata11[2] = {2,-0.1};
  Matrix M10(2,3,testdata10);  
  Matrix M11(2,1,testdata11);  
  M4.copy(M1);
  M4.toMultrowwise(M11);
  unit_assert( "multrowwise()",   M4 == M10 );
  D testdata12[6] = {2,1,0, 8, 2.5, 0 };
  D testdata13[3] = {2, 0.5, 0};
  M10.set(2,3,testdata12);  
  Matrix M12(3,1,testdata13);  
  M4.copy(M1);
  M4.toMultcolwise(M12);
  unit_assert( "multrowwise()",   M4 == M10 );

  M3.copy(M1);  
  M4.copy(M1);
  M4.toTranspose();
  M5 = M3.multMT();
  M6.mult(M1,M4);
  unit_assert( "multMT() ",   M5 == M6 );
  M5 = M3.multTM();
  M6.mult(M4,M1);
  unit_assert( "multTM() ",   M5 == M6 );

  D testdata20[12]={1,2,3, 4,5,6, 1,2,3, 4,5,6 };
  const Matrix M20(4,3, testdata20);
  const Matrix M21 = M1.above(M1);
  unit_assert( "above() ",   M20 == M21 );
  
   
  
  unit_pass();
}

DEFINE_TEST( check_matrix_operators ) {  
  cout << "\n -[ Matrix Operators (+ - * ^)]-\n";
  D testdata[6]={1,2,3, 4,5,6 };
  const Matrix M1(2,3, testdata);
  D testdata2[6]={1,4, 2,5, 3,6 };
  const Matrix M2(3,2, testdata2);  
  unit_assert( "^T ",  (M1^T) == M2 );
  D testdata3[6]={2,4,6, 8,10,12};
  Matrix M4(2,3,testdata3);    
  unit_assert( "+  ", M1+M1 == M4 );
  unit_assert( "-  ", M1+M1-M1 == M1 );

  D testdata4[6]={-0.5, -1, -1.5,  -2, -2.5, -3};
  Matrix M3(2,3, testdata4);
  unit_assert( "* scalar", M1*(-0.5) == M3 );

  D testdata5[6] = {2, -1,  0, 3,  2, -2};
  D testdata6[4] = {8, -1,  20, -1};
  Matrix M5 (3,2, testdata5);
  Matrix M6(2,2,testdata6);
  unit_assert( "*  ", M1*M5 == M6 );

  unit_assert( "^1 ", (M1^1) == M1 );
  M3.toId();
  unit_assert( "^0=id ", (M1^0) == M3 );

  D testdata7[16] = {1,2,3,4, -4,2,1,3, 0.3,-0.9, 4, -3, 1,0.5,0.3,5.0};
  Matrix M7(4,4,testdata7);
  unit_assert( "^1 * ^-1=id ",   comparetoidentity(M7*(M7^-1)) );
  unit_pass();
}

DEFINE_TEST( speed ) {  
  cout << "\n -[ Speed: Inverion]-\n";  
#ifndef NDEBUG
  cout << "   DEBUG MODE! use -DNDEBUG -O3 (not -g) to get full performance\n";
#endif
  Matrix M1;
  srand(time(0));
  D testdata0[9] = {1,2, -4,2}; 
  Matrix M2(2,2,testdata0);
  UNIT_MEASURE_START("2x2 Matrix inversion", 100000)
    M1 = (M2^-1);
  UNIT_MEASURE_STOP("");
  unit_assert( "validation", comparetoidentity(M1*M2));
  /* LU version:  555648/s */
  /* Explicit  : 1428775/s */
  D testdata1[9] = {1,2,3, -4,2,1, 0.3,-0.9}; 
  Matrix M3(3,3,testdata1);
  UNIT_MEASURE_START("3x3 Matrix inversion", 100000)
    M1 = (M3^-1);
  UNIT_MEASURE_STOP("");
  unit_assert( "validation", comparetoidentity(M1*M3));

  D testdata2[16] = {1,2,3,4, -4,2,1,3, 0.3,-0.9, 4, -3, 1,0.5,0.3,5.0};
  Matrix M4(4,4,testdata2);
  UNIT_MEASURE_START("4x4 Matrix inversion", 100000)
    M1 = (M4^-1);
  UNIT_MEASURE_STOP("");
  unit_assert( "validation", comparetoidentity(M1*M4));

  Matrix M20(20,20); 
  for (int i=0; i < M20.getM(); i++)  // define random values for initial matrix
    for (int j=0; j < M20.getN(); j++) {
      M20.val(i,j) = -22+(100. * rand())/RAND_MAX;
    }
  UNIT_MEASURE_START("20x20 Matrix inversion",1000)
    M1 = (M20^-1);  
  UNIT_MEASURE_STOP("");
  unit_assert( "validation", comparetoidentity(M1*M20));

  Matrix M200(200,200); 
  rand();  // eliminates the first (= zero) call
  for (int i=0; i < M200.getM(); i++)  // define random values for initial matrix
    for (int j=0; j < M200.getN(); j++) {
      M200.val(i,j) = -22+(100. * rand())/RAND_MAX;
    }
  UNIT_MEASURE_START("200x200 Matrix inversion",2)
    M1 = (M200^-1);  
  UNIT_MEASURE_STOP("");
  unit_assert( "validation", comparetoidentity(M1*M200));

  cout << "\n -[ Speed: Other Operations]-\n";    
  UNIT_MEASURE_START("20x20 Matrix multiplication with assignment",5000)
    M1 = M20*M20;  
  UNIT_MEASURE_STOP("");
  UNIT_MEASURE_START("20x20 Matrix addition with assignment",100000)
    M1= (M1 + M20);  
  UNIT_MEASURE_STOP("");
  UNIT_MEASURE_START("20x20 Matrix inplace addition",100000)
    M1 += M20;  
  UNIT_MEASURE_STOP("");
  UNIT_MEASURE_START("20x20 Matrix transposition",100000)
    M1 += M20;  
  UNIT_MEASURE_STOP("");

  unit_pass();  
}

UNIT_TEST_RUN( "Matrix Tests" )
  ADD_TEST( check_creation )
  ADD_TEST( check_vector_operation )
  ADD_TEST( check_matrix_operation )
  ADD_TEST( check_matrix_operators )
  ADD_TEST( speed )

  UNIT_TEST_END

#endif // UNITTEST
