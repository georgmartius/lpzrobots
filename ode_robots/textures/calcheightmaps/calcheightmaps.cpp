/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/
#include <iostream>
using namespace std;

#include "imageppm.h"

double sqr(double p) { return p*p;}
double min(double a,double b) { return a<b ? a : b;}
double max(double a,double b) { return a>b ? a : b;}

#define IMAGE_SIZE 256
#define TEXTURE_STEPS 8
#define TEXTURE_MIN 128
#define TEXTURE_MAX 255

/// texture coding that uses TEXTURE_MIN, TEXTURE_MAX
void codeGrayTex(double value, unsigned char* data);
/// heightmap coding where r g b get the same value 
void codeEqual(double value, unsigned char* data);
// heightmap coding where r+g+b = heightvalue
void codeSum(double value, unsigned char* data);
/// heightmap coding where r<<16+g<<8+b = heightvalue
void codeLowMidHigh(double value, unsigned char* data);

double calcMacroSpheres(double x, double y){  
  double s=4.4;
  double p1 = sqr((x-0.33)*s)+sqr((y-0.33)*s);
  double p2 = sqr((x-0.54)*s)+sqr((y-0.36)*s);
  double p3 = sqr((x-0.67)*s)+sqr((y-0.67)*s);

  return min(1,min(min(p1,p2),p3));
}

double calcSingleParabel(double x, double y){  
  double s=2.1;
  double p1 = sqr((x-0.5)*s)+sqr((y-0.5)*s);
  return min(1,p1);
}

/// function that returns an value of the height map at the position (x,y)
// range: 0<= x < 1; 0<= y < 1; 
// can return any number. The map is scaled to 0 - 255 later.
double (*fun)(double, double) = &calcMacroSpheres;

/// function that codes the double value into the r g b values of the given data array
void (*code)(double value, unsigned char* data) = &codeGrayTex;

int main(){
  double value;
  double minimum=1e100;
  double maximum=-1e100;
  int size=IMAGE_SIZE;
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

void codeGrayTex(double value, unsigned char* data){
  // discretised and scaled
  int v = (int(value*TEXTURE_STEPS))*(TEXTURE_MAX-TEXTURE_MIN)/TEXTURE_STEPS + TEXTURE_MIN;

  data[0] = v;
  data[1] = v;
  data[2] = v;  
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
