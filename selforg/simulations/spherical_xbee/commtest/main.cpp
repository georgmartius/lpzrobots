// test program for communication with xbee and atmega
//  atmega programm under: avr/projects/comm_test
// Main code is in serial_unix.cpp in run() function

#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <iterator>

#include "serial_unix.h"

// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}

int main(int argc, char** argv){
  int verboseMode=0;
  const char* port = "/dev/ttyS0";
  int baud = 38400;

  if(contains(argv,argc,"-v")!=0) verboseMode=1;
  if(contains(argv,argc,"-vv")!=0) verboseMode=2;
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g] [-f] [-v[v]]\n",argv[0]);
    printf("\t-h\tdisplay this help\n");
    printf("\t-b baud\tset baud rate\n");
    printf("\t-v\tenable verbose mode\n\t-p port\tuse give serial port (/dev/ttyS0)\n");
    exit(0);
  }
  int index = contains(argv,argc,"-p");
  if(index && index<argc){
    port = argv[index];
    cout << "use port " << port << endl;
  }
  index = contains(argv,argc,"-b");
  if(index && index<argc){
    baud = atoi(argv[index]);
    cout << "use baud rate " << baud << endl;
  }

  CSerialThread* s = new CSerialThread(port, baud, false);

  s->start();
  while(1);

  fprintf(stderr,"Serial thread terminated\n");
  delete s;
  return 0;
}
