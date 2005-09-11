#ifndef __GRABFRAME_H
#define __GRABFRAME_H

#include <stdio.h>

typedef struct VideoStream{
  VideoStream(){file=0; buf=0; revbuf=0;}
  FILE* file;
  int w;
  int h;
  unsigned char* buf;
  unsigned char* revbuf;
};

VideoStream openVideoStream(const char* filename);
void closeVideoStream(VideoStream& f);
bool grabAndWriteFrame(VideoStream f);



#endif
