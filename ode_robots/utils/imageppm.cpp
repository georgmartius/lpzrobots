#include "imageppm.h"
#include "stdio.h"

static int readNumber (char *filename, FILE *f)
{
  int c,n=0;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) printf ("unexpected end of file in \"%s\"",filename);
    if (c >= '0' && c <= '9') n = n*10 + (c - '0');
    else {
      ungetc (c,f);
      return n;
    }
  }
}


static void skipWhiteSpace (char *filename, FILE *f)
{
  int c,d;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) printf ("unexpected end of file in \"%s\"",filename);

    // skip comments
    if (c == '#') {
      do {
	d = fgetc(f);
	if (d==EOF) printf ("unexpected end of file in \"%s\"",filename);
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

int ImagePPM::loadImage(char*filename)
{
  FILE *f = fopen (filename,"rb");
  if (!f) 
  {  printf ("Can't open image file `%s'", filename);
     return 1;
  }

  // read in header
  if (fgetc(f) != 'P' || fgetc(f) != '6')
    printf ("image file \"%s\" is not a binary PPM (no P6 header)",filename);
  skipWhiteSpace (filename,f);

  // read in image parameters
  image_width = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  image_height = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  int max_value = readNumber (filename,f);

  // check values
  if (image_width < 1 || image_height < 1)
    printf ("bad image file \"%s\"",filename);
  if (max_value != 255)
    printf ("image file \"%s\" must have color range of 255",filename);

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
  if (fread( image_data, image_width*image_height*3, 1, f) != 1)
    printf ("Can not read data from image file `%s'",filename);
  fclose (f);
  return 0;
}


ImagePPM::~ImagePPM()
{
  if(image_data) delete[] image_data;
}
