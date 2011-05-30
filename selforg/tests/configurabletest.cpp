/***************************************************************************
                          tests.cpp  -  description
                             -------------------
    email                : georg.martius@web.de
***************************************************************************/
// Tests for configurables
//
/***************************************************************************/

#include "unit_test.hpp"

#include <selforg/configurable.h>

#include <stdlib.h>
#include <string.h>


using namespace std;
// globals to be ref'ed to

struct Dat {
  double d;
  int i;
  bool b;
  double pla;  
  bool operator == (const Dat& dat){
    return d==dat.d && i==dat.i && b == dat.b && pla==dat.pla; //memcmp(this, &dat, sizeof(Dat))==0;
  }
};


Configurable setupConfigable(Dat& d, const char* name){
  Configurable c(name,"0");
  c.addParameterDef("d",&d.d,0,"double");
  c.addParameterDef("i",&d.i,0,"int");
  c.addParameterDef("b",&d.b,false,"bool");
  c.addParameterDef("pla",&d.pla,1.0,"a pretty long description (causes line break), a pretty long description (causes line break)....");
  return c;
}

void setValues(Configurable& c){
  c.setParam("d",0.1234, false);
  c.setParam("pla", -123456, false);
  c.setParam("i",-123, false);
  c.setParam("b",true, false);
}

UNIT_TEST_DEFINES


DEFINE_TEST( CheckSetGet ) {  
  cout << "\n -[ Check Set and Get ]-\n";
  Dat d;
  Configurable c = setupConfigable(d,"c");
  setValues(c);
  unit_assert( "set double", c.getParam("d")  == 0.1234 );
  unit_assert( "set double-", c.getParam("pla")  == -123456 );
  unit_assert( "set int   ", c.getParam("i")  == -123 );
  unit_assert( "set bool  ", c.getParam("b")  == true );
  unit_pass();
}


DEFINE_TEST( store_restore ) {  
  cout << "\n -[ Store and Restore ]-\n";  
  Dat d,d1,d2,d3;
  Configurable c = setupConfigable(d,"c");
  Configurable c1 = setupConfigable(d1,"c1");
  Configurable c2 = setupConfigable(d2,"c2");
  Configurable c3 = setupConfigable(d3,"c3");
  c1.addConfigurable(&c2);
  c.addConfigurable(&c1);
  c.addConfigurable(&c3);
  setValues(c);
  setValues(c2);
  //  c.print(stdout,0,80);
  unit_assert("store   ", c.storeCfg("configurablestore"));
  
  Dat bd,bd1,bd2,bd3;
  Configurable bc = setupConfigable(bd,"c");
  Configurable bc1 = setupConfigable(bd1,"c1");
  Configurable bc2 = setupConfigable(bd2,"c2");
  Configurable bc3 = setupConfigable(bd3,"c3");    
  bc1.addConfigurable(&bc2);
  bc.addConfigurable(&bc1);
  bc.addConfigurable(&bc3);
  unit_assert("restore ", bc.restoreCfg("configurablestore"));
  // validate:
  unit_assert("validate ", d==bd && d1==bd1 && d2==bd2 && d3==bd3 );
  bc.print(stdout,0,80); 
  
  unit_pass();  
}

// DEFINE_TEST( store_restore ) {  
//   cout << "\n -[ Store and Restore ]-\n";  


//   unit_pass();  
// }



UNIT_TEST_RUN( "Configurable Tests" )
  ADD_TEST( CheckSetGet )
  ADD_TEST( store_restore )

  UNIT_TEST_END

