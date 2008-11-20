// -*- C++ -*-
#ifdef GNUPLOT_ONLY_INCLUDES

#include <unistd.h>

#else

static FILE* OpenGnuplot(int w=400,int h=300, int x=-1, int y=-1){
  char cmd[256];
  if(x==-1 || y==-1)
    sprintf(cmd, "gnuplot -geometry %ix%i -noraise >/dev/null 2>/dev/null", w, h);
  else 
    sprintf(cmd, "gnuplot -geometry %ix%i+%i+%i -noraise >/dev/null 2>/dev/null", w, h, x, y);
  
  return popen(cmd,"w");
  //return popen("gnuplot -geometry 400x300","w");
  /*char b[100];
  sprintf(b,"gnup%i",rand()%1000);
  return fopen(b,"w");*/
}
static void CloseGnuplot(FILE* pipe){
    pclose(pipe);
}

#endif
