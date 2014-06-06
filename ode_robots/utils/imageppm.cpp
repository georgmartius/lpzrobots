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
#include "imageppm.h"
#include <stdio.h>
#include <iostream>
#include <string>

static int readNumber (const std::string& filename, FILE *f)
{
  int c,n=0;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) std::cerr << "unexpected end of file in '" << filename << "'" << std::endl;
    if (c >= '0' && c <= '9') n = n*10 + (c - '0');
    else {
      ungetc (c,f);
      return n;
    }
  }
}


static void skipWhiteSpace (const std::string& filename, FILE *f)
{
  int c,d;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) std::cerr << "unexpected end of file in '" << filename << "'" << std::endl;

    // skip comments
    if (c == '#') {
      do {
        d = fgetc(f);
        if (d==EOF) std::cerr << "unexpected end of file in '" << filename << "'" << std::endl;
      } while (d != '\n');
      continue;
    }

    if (c > ' ') {
      ungetc (c,f);
      return;
    }
  }
}



ImagePPM::ImagePPM ()
{   image_data = 0;
}

ImagePPM::ImagePPM (int width, int height, unsigned char* data){
  image_width = width;
  image_height = height;
  image_data = data;
}


int ImagePPM::loadImage(const std::string& filename)
{
  FILE *f = fopen (filename.c_str(),"rb");
  if (!f) {
    std::cerr << "Can't open image file '" <<  filename <<  "'" << std::endl;
    return 0;
  }

  // read in header
  if (fgetc(f) != 'P' || fgetc(f) != '6')
    std::cerr << "image file ist not binary PPM (no P6 header) '" <<  filename <<  "'" << std::endl;
  skipWhiteSpace (filename,f);

  // read in image parameters
  image_width = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  image_height = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  int max_value = readNumber (filename,f);

  // check values
  if (image_width < 1 || image_height < 1)
    std::cerr << "bad image file '" <<  filename <<  "'" << std::endl;
  if (max_value != 255)
    std::cerr << "image file '" <<  filename <<  "' must have color range 255" << std::endl;

  // read either nothing, LF (10), or CR,LF (13,10)
  int c = fgetc(f);
  if (c == 10) {
    // LF
  }
  else if (c == 13) {
    // CR
    c = fgetc(f);
    if (c != 10) ungetc (c,f);
  }
  else ungetc (c,f);

  // read in rest of data
  image_data = new unsigned char [image_width*image_height*3];
  if (fread( image_data, image_width*image_height*3, 1, f) != 1){
    std::cerr << "Can't read data from image file '" <<  filename <<  "'" << std::endl;
    return 0;
  }
  fclose (f);
  return 1;
}


int ImagePPM::storeImage(const std::string& filename) {
  FILE *f = fopen (filename.c_str(),"wb");
  if (!f) {
    std::cerr << "Can't open image file '" <<  filename <<  "'" << std::endl;
    return 0;
  }

  // write header
  fprintf(f,"P6\n");
  fprintf(f,"# CREATOR ImagePPM class of lpzrobots project\n");
  fprintf(f,"%i %i\n", image_width, image_height);
  fprintf(f,"255\n");

  // write data
  if (fwrite( image_data, image_width*image_height*3, 1, f) != 1){
    std::cerr << "Can't write to image file '" <<  filename <<  "'" << std::endl;
    fclose (f);
    return 0;
  }
  fclose (f);
  return 1;
}


ImagePPM::~ImagePPM()
{
  if(image_data) delete[] image_data;
}
