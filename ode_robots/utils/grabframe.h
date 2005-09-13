#ifndef __GRABFRAME_H
#define __GRABFRAME_H

#include <stdio.h>

typedef struct VideoStream{
  VideoStream(){filename=0; buf=0; revbuf=0; opened=false;}
  bool opened;
  char* filename;
  int w;
  int h;
  unsigned char* buf;
  unsigned char* revbuf;
  long int counter;
  
};

VideoStream openVideoStream(const char* filename);
void closeVideoStream(VideoStream& f);
bool grabAndWriteFrame(VideoStream& f);



#endif
