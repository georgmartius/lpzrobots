#include <stdio.h>
#include <signal.h>
#include <iostream>
using namespace std;
#include "cmdline.h"

// Commandline interface stuff
void showParams(const ConfigList& configs, FILE* f /*= 0*/, const char* lineprefix /*= 0*/)
{
  if(!f) f=stdout;
  for(vector<Configurable*>::const_iterator i=configs.begin(); i != configs.end(); i++){
    (*i)->print(f, lineprefix);
  }
}


/// internals
int Control_C=0;

void cmd_handler_exit(void){
  signal(SIGINT,SIG_DFL);    
  Control_C=0;
}

void cmd_handler_cleanup(void){
  signal(SIGINT,SIG_DFL);
}

void control_c(int i){
  cmd_handler_exit();
  Control_C++ ;
  // if (Control_C > 100)exit(0);
}

void cmd_handler_init(){
  signal(SIGINT,control_c);
  atexit(cmd_handler_cleanup);
  Control_C=0;  
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

