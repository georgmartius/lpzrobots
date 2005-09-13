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
  assert (filename);
  stream.filename = new char[strlen(filename) + 1];
  strcpy(stream.filename, filename);
  stream.buf      = new unsigned char[stream.w*stream.h*3];
  stream.revbuf   = new unsigned char[stream.w*stream.h*3];
  stream.counter  = 0;
  stream.opened = true;

//   assert (filename);
//   stream.file=fopen(filename, "wb");
//   if(!stream.file){
//     fprintf(stderr,"Cannot open file %s for writing",filename);    
//   }
  return stream;
}

void closeVideoStream(VideoStream& stream){
  if(stream.buf) delete[] stream.buf;
  if(stream.revbuf) delete[] stream.revbuf;
  if(stream.filename) delete[] stream.filename;
  stream.filename = 0;
  stream.buf    = 0;
  stream.revbuf = 0;
  stream.opened = false;
}

bool grabAndWriteFrame(VideoStream& stream){
  if(!stream.opened) return false;
  char name[128];
  FILE *file;
  bool ok = getGLFrameBuffer(stream.buf, stream.w, stream.h);
  if (ok) {
    for (int y=0; y<stream.h; y++) {
      int chunk = 3 * stream.w;
      memcpy(stream.revbuf + y*chunk, stream.buf + (stream.h - 1 - y)*chunk, chunk);
    }
    sprintf(name,"%s_%06ld.ppm", stream.filename, stream.counter);
    stream.counter++;
    file=fopen(name, "wb");
    if(!file){
      fprintf(stderr,"Cannot open file %s for writing",stream.filename);    
    }else{
      fprintf(file,"P6 %d %d 255\n", stream.w, stream.h);
      fwrite(stream.revbuf, stream.w*stream.h*3, 1, file);
      fclose(file);
    }
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
 
