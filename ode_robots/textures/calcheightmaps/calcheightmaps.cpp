#include <iostream>
using namespace std;

#include "imageppm.h"


double sqr(double p) { return p*p;}
double min(double a,double b) { return a<b ? a : b;}
double max(double a,double b) { return a>b ? a : b;}

void codeSum(double value, unsigned char* data);
void codeEqual(double value, unsigned char* data);
void codeLowMidHigh(double value, unsigned char* data);

double calcMacroSpheres(double x, double y){  
  double s=4.4;
  double p1 = sqr((x-0.33)*s)+sqr((y-0.33)*s);
  double p2 = sqr((x-0.58)*s)+sqr((y-0.36)*s);
  double p3 = sqr((x-0.67)*s)+sqr((y-0.67)*s);

  return min(1,min(min(p1,p2),p3));
}

/// function that returns an value of the height map at the position (x,y)
// range: 0<= x < 1; 0<= y < 1; 
// can return any number. The map is scaled to 0 - 255 later.
double (*fun)(double, double) = &calcMacroSpheres;

/// function that codes the double value into the r g b values of the given data array
void (*code)(double value, unsigned char* data) = &codeSum;

int main(){
  double value;
  double minimum=1e100;
  double maximum=-1e100;
  int size=256;
  double *array = new double[size*size];
  for(int x=0; x<size; x++){
    for(int y=0; y<size; y++){
      value = fun(double(x)/size, double(y)/size);
      minimum = min(value, minimum);
      maximum = max(value, maximum);
      array[x+y*size]=value;
    }
  }
  
  unsigned char* image = new unsigned char [size*size*3];
  double range = maximum - minimum;
  
  // normalise and write to final bitmap buffer
  for(int i=0; i<size*size; i++){
    value = array[i];
    code((value + minimum)/range, &image[3*i]);
  }
  
  ImagePPM img = ImagePPM(size, size, image);
  if(!img.storeImage("heightmap.ppm")) {
    fprintf(stderr,"Shit\n");
  };

  delete[] array;

  return 0;
}

void codeEqual(double value, unsigned char* data){
  int v = int(255.0*value);
  data[0] = v;
  data[1] = v;
  data[2] = v;  
}

void codeSum(double value, unsigned char* data){
  int v = int(768.0*value);
  data[0] = v/3 + v%256;
  data[1] = v/3;
  data[2] = v/3;  
}

void codeLowMidHigh(double value, unsigned char* data){
  long v = long(65536.0*value);
  data[0] = (v & 0xFF0000)  >> 16;
  data[1] = (v & 0x00FF00) >> 8;
  data[2] = (v & 0x0000FF);
}
