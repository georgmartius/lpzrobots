#include <GL/gl.h>
#include <assert.h>
#include <string.h>
#include "grabframe.h"

bool getGLFrameBuffer( unsigned char *buf, int w, int h);

VideoStream openVideoStream(const char* filename){
  int vals[4];
  glGetIntegerv(GL_VIEWPORT,vals);
  
  VideoStream stream;
  stream.w    = vals[2];
  stream.h    = vals[3];
  stream.file = 0;
  stream.buf  = new unsigned char[stream.w*stream.h*3];
  stream.revbuf = new unsigned char[stream.w*stream.h*3];

  assert (filename);
  stream.file=fopen(filename, "wb");
  if(!stream.file){
    fprintf(stderr,"Cannot open file %s for writing",filename);    
  }
  return stream;
}

void closeVideoStream(VideoStream& stream){
  if(stream.file) fclose(stream.file);
  if(stream.buf) delete stream.buf;
  if(stream.revbuf) delete stream.revbuf;
  stream.file   = 0;
  stream.buf    = 0;
  stream.revbuf = 0;
}

bool grabAndWriteFrame(VideoStream stream){
  if(!stream.file) return false;
  bool ok = getGLFrameBuffer(stream.buf, stream.w, stream.h);
  if (ok) {
    for (int y=0; y<stream.h; y++) {
      int chunk = 3 * stream.w;
      memcpy(stream.revbuf + y*chunk, stream.buf + (stream.h - 1 - y)*chunk, chunk);
    }
    fwrite(stream.revbuf, stream.w*stream.h*3, 1, stream.file);
  }
  return true;
}


bool getGLFrameBuffer( unsigned char *buf, int w, int h){    
  if (!buf)
    return false;
  glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE,(GLvoid*)buf);
  return true;
} 

 
//  And finally, to create mpegs, (I made mpeg1), you simply
//  use the mpeg_encode with this cfg file encode.stats:
 
//  PATTERN         ibbpbbpbbpbbpbb
 
//  SLICES_PER_FRAME        1
 
//  OUTPUT nb.mpg
 
//  GOP_SIZE        50
 
//  INPUT_DIR .
 
 
//  INPUT_CONVERT   *
 
//  INPUT
//  nb*.ppm [0000-3181]
//  END_INPUT
 
//  BASE_FILE_FORMAT PPM
 
//  ERROR           MAD
 
 
//  PIXEL HALF
 
//  # means +/- this many pixels
//  RANGE           10
 
//  PSEARCH_ALG     EXHAUSTIVE
//  BSEARCH_ALG     CROSS2
//  #IQSCALE         8
//  #PQSCALE         10
//  #BQSCALE         25
 
//  IQSCALE         26
//  PQSCALE         26
//  BQSCALE         26
 
//  REFERENCE_FRAME ORIGINAL
 
