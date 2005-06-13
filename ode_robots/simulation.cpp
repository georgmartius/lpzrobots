#include <stdlib.h>
#include <signal.h>
#include <iostream>
using namespace std;
#include "simulation.h"

void showParams(Configurable** configs, int len){
  paramkey* keys;
  paramval* vals;
  for(int i=0; i < len; i++){
    int pnum = configs[i]->getParamList(keys,vals);
    printf("Parameters of %s\n", configs[i]->getName());
    for(int j=0; j < pnum; j++) {
      printf(" %s=%f\n", keys[j], vals[j]);
    }
    free(keys);
    free(vals);
  }
}

void changeParams(Configurable** configs, int len){
  char buffer[1024];
  std::cout << "Type: Parameter=Value\n";
  fgets( buffer, 1024, stdin);
  if ( strchr(buffer,'?')!=0){
    showParams(configs, len);
    return;
  }

  char *p = strchr(buffer,'=');
  if (p){
    *p=0; // terminate key string 
    double v=strtod(p+1,0);
    for(int i=0; i < len; i++){
      if (configs[i]->setParam(buffer,v))
	printf(" %s=%f \n", buffer, configs[i]->getParam(buffer));
    }
  }
}

/// internals
int Control_C=0;

void control_c(int i){
  Control_C++ ;
  // if (Control_C > 100)exit(0);
}

void cmd_handler_exit(void){
  signal(SIGINT,SIG_DFL);
  Control_C=0;
}

void cmd_handler_init(){
  signal(SIGINT,control_c);
  atexit(cmd_handler_exit);
}

bool control_c_pressed(){
  return Control_C!=0;
}

void cmd_begin_input(){
  cmd_handler_exit();
}

void cmd_end_input(){
  cmd_handler_init();  
}


