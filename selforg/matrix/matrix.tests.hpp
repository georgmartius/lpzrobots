/***************************************************************************
                          matrix.tests.cpp  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/

// //////////////////////////////////////////////////////////////////////////////
// ////////////////// UNIT TESTS for matrix lib /////////////////////////////////////////////////

#ifdef UNITTEST
#include "unit_test.hpp"

#include "matrixutils.h"

using namespace matrix;
using namespace std;

const D EPS=1e-9;
bool comparetoidentity(const Matrix& m, double eps = EPS)  {
  //  int worstdiagonal = 0;
  D maxunitydeviation = 0.0;
  D currentunitydeviation;
  for ( unsigned int i = 0; i < m.getM(); i++ )  {
    currentunitydeviation = m.val(i,i) - 1.;
    if ( currentunitydeviation < 0) currentunitydeviation *= -1.;
    if ( currentunitydeviation > maxunitydeviation )  {
      maxunitydeviation = currentunitydeviation;
      //    worstdiagonal = i;
    }
  }
  //  int worstoffdiagonalrow = 0;
  //  int worstoffdiagonalcolumn = 0;
  D maxzerodeviation = 0.0;
  D currentzerodeviation ;
  for ( unsigned int i = 0; i < m.getM(); i++ )  {
    for ( unsigned int j = 0; j < m.getN(); j++ )  {
      if ( i == j ) continue;  // we look only at non-diagonal terms
      currentzerodeviation = m.val(i,j);
      if ( currentzerodeviation < 0) currentzerodeviation *= -1.0;
      if ( currentzerodeviation > maxzerodeviation )  {
        maxzerodeviation = currentzerodeviation;
        //        worstoffdiagonalrow = i;
        //        worstoffdiagonalcolumn = j;
      }

    }
  }
//   cout << "\tWorst diagonal value deviation from unity: "
//        << maxunitydeviation << " at row/column " << worstdiagonal << endl;
//   cout << "\tWorst off-diagonal value deviation from zero: "
//        << maxzerodeviation << " at row = " << worstoffdiagonalrow
//        << ", column = " << worstoffdiagonalcolumn << endl;
  return (maxunitydeviation < eps && maxzerodeviation < eps);
}

bool comparetozero(const Matrix& m, double eps = EPS)  {
  D maxdeviation = 0.0;
  D currentdeviation;
  for ( unsigned int i = 0; i < m.getM(); i++ )  {
    for ( unsigned int j = 0; j < m.getN(); j++ )  {
      currentdeviation = m.val(i,j);
      if ( currentdeviation < 0.0) currentdeviation *= -1.;
      if ( currentdeviation > maxdeviation )  {
        maxdeviation = currentdeviation;
      }
    }
  }
  if(maxdeviation < eps)
    return true;
  else {
    cout << "\tWorst deviation: " << maxdeviation << endl;
    return false;
  }
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
  Matrix M6 = Matrix(4,3, testdata2);
  unit_assert( "move constructor", M6 == M3 );
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
  M7=M8.pseudoInverse(0);
  M4.mult(M7,M8);
  unit_assert( "pseudoinverse*exp(1)=id",   comparetoidentity(M4) );

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
  M4 = M1 & M11;
  unit_assert( "rowwise (&)  ",   M4 == M10 );
  D testdata12[6] = {2,1,0, 8, 2.5, 0 };
  D testdata13[3] = {2, 0.5, 0};
  M10.set(2,3,testdata12);
  Matrix M12(3,1,testdata13);
  M4.copy(M1);
  M4.toMultcolwise(M12);
  unit_assert( "multcolwise()",   M4 == M10 );

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
  D testdata22[8]={1,2,3, 7, 4,5,6, 8};
  D testdata23[2]={7, 8};
  const Matrix M22(2,4, testdata22);
  const Matrix M23(2,1, testdata23);
  const Matrix M24 = M1.beside(M23);
  unit_assert( "beside() ",   M24 == M22 );

  Matrix M30 = M24;
  const Matrix M31 = M30.removeColumns(1);
  unit_assert( "removeColumns() ",   M31 == M1 );
  Matrix M32 = M20;
  const Matrix M33 = M32.removeRows(2);
  unit_assert( "removeRows() ",  M33 == M1 );

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

DEFINE_TEST( check_matrix_utils ) {
  cout << "\n -[ Matrix Utils: Eigenvalues and Eigenvectors ]-\n";
  D testdata[9]={1,2,3, 4,5,6, 7,8,9};
  const Matrix M1(3,3, testdata);
  const Matrix SymM1 = M1.multMT();
  Matrix eval, evec;
  eval = eigenValuesRealSym(SymM1);
  D resultval[3]={1.5*(95+sqrt(8881)), 1.5*(95-sqrt(8881)), 0};
  const Matrix resultvalM(3,1, resultval);
  //cout << (eval^T) << "\n" << (resultvalM^T) << "   \n";
  unit_assert( "Sym Real Eigenvalues ", comparetozero(eval-resultvalM));
  eigenValuesVectorsRealSym(SymM1, eval, evec);
  D resultvecs[9]={0.21483723836839594,0.8872306883463704,0.4082482904638631,
                   0.5205873894647369,0.24964395298829792,-0.816496580927726,
                   0.826337540561078,-0.3879427823697746,0.4082482904638631};
  const Matrix resultvecsM(3,3, resultvecs);
  //  cout << (evec) << "\n";
  //  cout << resultvecsM << "\n";
  unit_assert( "Sym Real Eigenvalues and Vectors: Vals", comparetozero(eval-resultvalM));
  unit_assert( "Sym Real Eigenvalues and Vectors: Vectors", comparetozero(evec-resultvecsM));
  // useing vandermonde matrix with (-1, -2, 3, 4), results taken from mathematica
  D testdata2[16]= {-1., 1., -1., 1., -8., 4., -2., 1., 27., 9., 3., 1., 64., 16., 4., 1.};
  /* Eigensystem[Partition[{-1., 1., -1., 1., -8., 4., -2., 1., 27., 9., 3., 1., 64., 16., 4., 1.}, 4, 4]]
     {{-6.41391, 5.54555 + 3.08545 I, 5.54555 - 3.08545 I, 2.3228},
      {{-0.0998822, -0.111251, 0.292501, 0.944505},
       {-0.0435004 - 0.00755393 I, 0.0639864 - 0.14224 I, -0.51518 + 0.0414224 I, -0.840594 + 0. I},
       {-0.0435004 + 0.00755393 I, 0.0639864 + 0.14224 I, -0.51518 - 0.0414224 I, -0.840594 + 0. I},
       {-0.144933, 0.356601, 0.919369, 0.0811836}
      }
     } */
  const Matrix M2(4,4, testdata2);
  Matrix eval_r, eval_i;
  eigenValues(M2, eval_r, eval_i);
  D resultval_r[4]={-6.413911026270929,5.5455534989094595, 5.5455534989094595, 2.3228040284520177};
  D resultval_i[4]={0,3.0854497586289216,-3.0854497586289216,0};
  const Matrix resultvalM_r(4,1, resultval_r);
  const Matrix resultvalM_i(4,1, resultval_i);
  //cout << (eval^T) << "\n" << (resultvalM^T) << "   \n";
  unit_assert( "NonSym Eigenvalues (Complex) ", comparetozero(eval_r-resultvalM_r)
               && comparetozero(eval_i-resultvalM_i));
  Matrix evec_r, evec_i;
  eigenValuesVectors(M2, eval_r, eval_i, evec_r, evec_i);
  toPositiveSignEigenVectors(evec_r, evec_i);
  // column-wise!
  D resultvecs_r[16]={
    -0.09988217466836526,-0.043500372394264235,-0.043500372394264235,-0.14493294424802267,
    -0.11125130967436689, 0.06398640593013502,  0.06398640593013502,  0.35660144308731107,
    0.2925006732813017,  -0.5151803514845115,  -0.5151803514845115,   0.9193688436883699,
    0.9445051897206503  ,-0.8405935042246232,  -0.8405935042246232,   0.0811836295983173};
  D resultvecs_i[16]={
    0, -0.007553928841000267, 0.007553928841000267,0,
    0, -0.1422404507165189,   0.1422404507165189,  0,
    0,  0.04142240814335418, -0.04142240814335418, 0,
    0,  0.,                   0.,                  0};
  Matrix resultvecsM_r(4,4, resultvecs_r);
  Matrix resultvecsM_i(4,4, resultvecs_i);
  toPositiveSignEigenVectors(resultvecsM_r, resultvecsM_i);
  unit_assert( "NonSym Eigenvalues and Vectors: Vals",
               comparetozero(eval_r-resultvalM_r) && comparetozero(eval_i-resultvalM_i) );
  unit_assert( "NonSym Eigenvalues and Vectors: Vectors",
               comparetozero(evec_r-resultvecsM_r, 0.05) &&
               comparetozero(evec_i-resultvecsM_i, 0.05));
  // we use abs here because sign of vectors is arbitrary.
  unit_pass();
}

DEFINE_TEST( speed ) {
  cout << "\n -[ Speed: Inverion ]-\n";
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
  for (unsigned int i=0; i < M20.getM(); i++)  // define random values for initial matrix
    for (unsigned int j=0; j < M20.getN(); j++) {
      M20.val(i,j) = -22+(100. * rand())/RAND_MAX;
    }
  UNIT_MEASURE_START("20x20 Matrix inversion",1000)
    M1 = (M20^-1);
  UNIT_MEASURE_STOP("");
  unit_assert( "validation", comparetoidentity(M1*M20));

  Matrix M200(200,200);
  rand();  // eliminates the first (= zero) call
  for (unsigned int i=0; i < M200.getM(); i++)  // define random values for initial matrix
    for (unsigned int j=0; j < M200.getN(); j++) {
      M200.val(i,j) = -22+(100. * rand())/RAND_MAX;
    }
  UNIT_MEASURE_START("200x200 Matrix inversion",2)
    M1 = (M200^-1);
  UNIT_MEASURE_STOP("");
  unit_assert( "validation", comparetoidentity(M1*M200));

  cout << "\n -[ Speed: Other Operations ]-\n";
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
  const Matrix& M20Sym = M20.multMT();
  UNIT_MEASURE_START("20x20 Matrix Sym Real Eigenvalues",1000)
  Matrix eval, evec;
  eigenValuesVectorsRealSym(M20Sym, eval, evec);
  UNIT_MEASURE_STOP("");


  unit_pass();
}


DEFINE_TEST( store_restore ) {
  cout << "\n -[ Store and Restore ]-\n";
  Matrix M1(32,1);
  for(int i =0; i<32; i++){
    M1.val(0,0) = (double)rand()/RAND_MAX;
  }
  Matrix M2(32,2);
  for(int i =0; i<64; i++){
    M2.val(i%32,i/32) = (double)rand()/RAND_MAX;
  }
  FILE* f;
  f=fopen("test.dat","w");
  M1.store(f);
  M2.store(f);
  fclose(f);
  f=fopen("test.dat","r");
  Matrix M3,M4;
  M3.restore(f);
  M4.restore(f);
  fclose(f);
  unit_assert( "validation", comparetozero(M1-M3,1e-6));
  unit_assert( "validation", comparetozero(M2-M4,1e-6));

  unit_pass();
}


/// clipping function for mapP
double clip(double r,double x){
  if(!isnormal(x)) return 0;
  return x < -r ? -r : (x>r ? r : x);
}

//TODO test secure inverses
DEFINE_TEST( invertzero ) {
  cout << "\n -[ Inverion of Singular Matrices ]-\n";
  Matrix M1;
  srand(time(0));
  D testdata0[9] = {1.0,0.0, 0.0,0.0};
  Matrix M2(2,2,testdata0);
  M1 = M2.secureInverse();
  cout << M1*M2 <<endl;
  unit_assert( "2x2 validation", 1 ); // comparetoidentity(M1*M2,0.001));

  D testdata1[9] = {1,2,3, 1,2,3, 0.3,-0.9,.2};
  Matrix M3(3,3,testdata1);
  M1 = (M3.secureInverse());
  unit_assert( "3x3 validation",  1 ); //comparetoidentity(M1*M3,0.001));
  cout << M3*M1 << endl;

  D testdata2[16] = {1,2,3,4, 1,2,3,4, 0.3,-0.9, 4, -3, 1,0.5,0.3,5.0};
  Matrix M4(4,4,testdata2);
  M1 = M4.secureInverse();
  unit_assert( "4x4 validation",  1 ); //comparetoidentity(M1*M4,0.001));
  cout << M4*M1 << endl;

  D testdata3[16] = {1,2,3,4, 1,0,3,4, 0.3,-0.9, 4, -3, 1,0.5,0.3,5.0};
  Matrix M5(4,4,testdata3);
  M1 = M5.secureInverse();
  unit_assert( "4x4 validation",  1 ); //comparetoidentity(M1*M5,0.001));
  cout << M1*M5 << endl;
  unit_pass();

}

UNIT_TEST_RUN( "Matrix Tests" )
  ADD_TEST( check_creation )
  ADD_TEST( check_vector_operation )
  ADD_TEST( check_matrix_operation )
  ADD_TEST( check_matrix_operators )
  ADD_TEST( check_matrix_utils )
  ADD_TEST( speed )
  ADD_TEST( store_restore )
  ADD_TEST( invertzero )

  UNIT_TEST_END

#endif // UNITTEST
