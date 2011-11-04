/***************************************************************************
 *   Copyright (C) 2004 by Patrick Audley                                  *
 *   paudley@blackcat.ca                                                   *
 *   modified by Georg Martius (georg.martius@web.de)                      *
 ***************************************************************************
# _______________
# VERSION HISTORY
#
# v1.0 (2004/12/28)
#  - prelimary version
# v1.1 (2005/05/30) (By Georg Martius
#  - improved output
#  - improved speed measurement
##
*/

/** 
 * @section C++ Unit Testing Framework
 * See implementation in
 * @ref unit_test.h
 *
 * @par
 * @section writing Writing Unit Tests
 * @par
 * Ideally, unit tests are written with the code they test.  The easiest way
 * to do this is to include all the unit tests at the bottom of each source
 * that they relate too.  It's also possible to create source files of nothing
 * but tests to expand coverage across multiple translation modules.
 * @par
 * Let's take a simple example:  we have a new function that adds two numbers.
 * @code
 * int addTwoNumbers( int a, int b ) {
 *   return a + b;
 * }
 * @endcode
 * To write a unit test that checks that @f$addTwoNumbers(x_1,x_2)=x_1+x_2@f$ we
 * would write the unit test like so:
 * @code
 * #ifdef UNITTEST
 * #include "unit_test.h"
 *
 * UNIT_TEST_DEFINES
 *
 * DEFINE_TEST( check_two_plus_two ) {
 *   unit_assert( "2+2=4", addTwoNumbers(2,2)==4 );
 * }
 *
 * UNIT_TEST_RUN( "addTwoNumbers Tests" )
 *   ADD_TEST( check_two_plus_two )
 * UNIT_TEST_END
 *
 * #endif // UNITTEST
 * @endcode
 * @par
 * Now we have a test suite defined that will only be compiled when we define UNITTEST.
 * UNIT_TEST_RUN actually creates a main() function that runs the tests so if we put
 * this code into a file (say add.cpp) then we can compile and run it like so:
@verbatim
# gcc -DUNITTEST -o unit_test_add add.cpp
# ./unit_test_add
---[ addTwoNumbers Tests ]---
  2+2=4: PASSED
@endverbatim
 * @par
 * So far so good, let's add a new test that we think will fail.
 * @code
 * #ifdef UNITTEST
 * #include "unit_test.h"
 *
 * UNIT_TEST_DEFINES
 *
 * DEFINE_TEST( check_two_plus_two ) {
 *   unit_assert( "2+2=4", addTwoNumbers(2,2)==4 );
 *   unit_pass();
 * }
 *
 * DEFINE_TEST( check_bogus ) {
 *   unit_assert( "1+5=9", addTwoNumbers(1,5)==9 );
 *   unit_pass();
 * }
 *
 * UNIT_TEST_RUN( "addTwoNumbers Tests" )
 *   ADD_TEST( check_negatives )
 * UNIT_TEST_END
 *
 * #endif // UNITTEST
 * @endcode
 * Running the unit_test now we get:
@verbatim
# gcc -DUNITTEST -o unit_test_add add.cpp
# ./unit_test_add
---[ addTwoNumbers Tests ]---
  2+2=4: PASSED
  1+5=9: FAILED
@endverbatim
 * @par
 * @section adding Integrating with Automake
 * @par
 * Automake has the ability to define testing targets that get run when
 * issue "make check" command.  Adding these tests are pretty straight
 * forward.  For the above we would add this to our Makefile.am:
@verbatim
TESTS = unit_test_add
noinst_PROGRAMS = unit_test_add
CLEANFILES = add_unit.cpp
unit_test_add_SOURCES = add_unit.cpp

%_unit.cpp: %.cpp
	$(CXX) -E -o $*_unit.cpp $*.C @CFLAGS@ -DUNITTEST=1
@endverbatim
 * To add addtional unit tests you just modify the first four lines.  For
 * example: to add a new unit test suite in the file sub.C we might do this.
@verbatim
TESTS = unit_test_add unit_test_sub
noinst_PROGRAMS = unit_test_add unit_test_sub
CLEANFILES = add_unit.cpp sub_unit.cpp
unit_test_add_SOURCES = add_unit.cpp
unit_test_sub_SOURCES = sub_unit.cpp
@endverbatim
 * @section Notes Implementation Notes
 * @par
 */

/**
 * @file unit_test.hpp Unit Testing framework for C++
 * @author Patrick Audley
 * @date December 2004
 */

#ifndef _UNIT_TEST_H
#define _UNIT_TEST_H

#ifdef UNITTEST
#include <iostream>
#include <vector>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <time.h>
#include <stdio.h>


struct rusage ruse;
extern int getrusage();
/** @brief Gets the current CPU time with microsecond accuracy.
 *  @returns microseconds since UNIX epoch
 */
inline double cputime( void ) {
  getrusage( RUSAGE_SELF, &ruse );
	return ( ruse.ru_utime.tv_sec + ruse.ru_stime.tv_sec + 1e-6 * (ruse.ru_utime.tv_usec + ruse.ru_stime.tv_usec ) );
}
/** @brief Calculates the transactions rate.
 *  @param run_time microsecond resolution run time
 *  @param transactions number of transactions handled in run_time seconds
 *  This is useful if you want to guarantee minimun transactional throughputs in unit tests.
 *  @warning This code is obviously very test platform dependent.
 */
inline double transactions_per_second( double run_time, unsigned long transactions ) {
	return (double)transactions / run_time;
}
/** @brief Prints to stdout the results of timing an event.
 *  @param msg to print with the numbers
 *  @param run_time microsecond resolution run time
 *  @param transactions number of transactions handled in run_time seconds, if 0 then transactional output is suppressed
 *  @warning This code is obviously very test platform dependent.
 */
inline void print_cputime( double run_time, unsigned long transactions = 0 ) {
  
  if( transactions == 0 ){
	printf("%7.3f seconds CPU time\n", run_time );
  }else{
    printf("(%li x):  %7.3f seconds CPU time\n", transactions, run_time );
    printf("      (%7.3f transactions/second)\n", 
	   transactions_per_second( run_time, transactions ) );
  }
}

/// typedef for unittest functions
typedef bool(*test_func)(void);
/// typedef for vectors of unittest functions
typedef std::vector< test_func > test_vector;

/** @brief Start of inline Unit Test definitions
 *  Use this to start the list of unit tests.  This should be followed
 *  by one or more DEFINE_TEST entries.
 */
#define UNIT_TEST_DEFINES \
  test_vector * add_test( test_func x ) { \
    static test_vector unit_tests; \
    if( x != NULL ) unit_tests.push_back( x ); \
    return &unit_tests; \
  }

/** @brief Start a new test definition
 *  @param test_name Name of the test - must be unique in this unit test suite.
 */
#define DEFINE_TEST(test_name) bool unit_test_##test_name (void)

/** @brief Adds a defined test to test run.
 *  @param test_name Test name of a previously defined test to add the the current suite.
 *  @sa DEFINE_TEST UNIT_TEST_RUN
 *  This should be called after UNIT_TEST_RUN for each defined test.
 */
#define ADD_TEST(test_name) add_test( &unit_test_##test_name );


/** @brief Starts the timer for CPU time measurement.   
 *  @param msg Message that should be printed
 *  @param times Number of times the Block should be executed
 *  @note Must be terminated with an UNIT_MEASURESTOP statement.
 */
#define UNIT_MEASURE_START(msg,times) \
  { std::cout << "  -> " <<  msg << std::flush; \
    double measure_t1 = cputime(); \
    int measure_times = times; \
    for(int measure_i=0; measure_i < times; measure_i++){

/** @brief Stops the timer for CPU time measurement and prints out result 
 *  @note Must be terminated with an UNIT_MEASURESTOP statement.
 */
#define UNIT_MEASURE_STOP(msg) \
    } /* end for */ \
    print_cputime(cputime()-measure_t1,measure_times); \
  }



/** @brief Start a Unit test run section.
 *  @param suite Name for this test suite.
 *  @note Must be terminated with an UNIT_TEST_END statement.
 */
#define UNIT_TEST_RUN( suite ) \
int main(void) { \
  bool result = true; \
  std::cout << "---[ " << suite << " ]--- " << std::endl;

/** @brief Use within a Unit Test to verify a condition.
 *  @warning Terminates test on failure.
 */
#define unit_assert( msg, cond ) \
  { \
  char buffer[40]; memset(buffer,32,40); \
  int pos = strlen(msg); \
  buffer[40- (pos>40 ? 40 : pos)]=0; \
  std::cout << "    " << msg << ": " << buffer << std::flush; \
  if( !(cond) ) { std::cout << "FAILED!" << std::endl; return false; } \
  std::cout << "PASSED" << std::endl; \
  }

/** @brief Use to end a unit test in success.
 *  @note Either unit_pass or unit_fail should end every test.
 */
#define unit_pass() return true;

/** @brief Use to end a unit test in failure.
 *  @note Either unit_pass or unit_fail should end every test.
 */
#define unit_fail() return false;

/** @brief Finish a Unit Test run section.
 */
#define UNIT_TEST_END \
  test_vector *_vector = add_test( NULL ); \
  for( unsigned short i = 0; i < _vector->size(); i++ ) { \
     bool testresult = (*(*_vector)[i])(); \
     if( result == true && testresult == false ) { result = false; } \
  } \
  return !result; \
}

#endif // UNITTEST

#endif // _UNIT_TEST_H
