#include <signal.h>
#include <stdio.h>
#include <string.h>
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

void changeParams(ConfigList& configs,
                  void (*onQuit)()){
  char buffer[1024];
  std::cout << "Type: Parameter=Value or ? for a listing or Ctrl-c for termination\n";
  fgets( buffer, 1024, stdin);
  if ( strchr(buffer,'?')!=0){
    showParams(configs);
    return;
  }

  char *p = strchr(buffer,'=');
  if (p){
    *p=0; // terminate key string
    double v=strtod(p+1,0);
    for(ConfigList::iterator i=configs.begin(); i != configs.end(); i++){
      if ((*i)->setParam(buffer,v))
        printf(" %s=\t%f \n", buffer, (*i)->getParam(buffer));
    }
  }
}

vector<string> splitString(const string& str, char seperator){
  vector<string> rv;
  string::const_iterator startword=str.begin();
  for(string::const_iterator i=str.begin(); i<str.end(); i++){
    if((i+1)==str.end())
      rv.push_back(string(startword, str.end()));
    else{
      if(*i==seperator){
        rv.push_back(string(startword, i));
        while( (*i)==seperator && i<str.end() ){ // skip multiple seperator
          i++;
        }
        startword=i;
      }
    }
  }
  return rv;
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

