/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.3  2005-11-09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
 #include <stdlib.h>
 #include <stdio.h>
 #include <assert.h>


/* mencoder mf:// *.png -mf w=800:h=600:fps=100:type=png -ovc lavc -lavcopts 
   vcodec=mpeg4 -oac copy -o output.avi -ffourcc DX50 */

void mencoder(const char* filename, int w, int h){
  char cmd[1024];
  printf("Should I invoke mencoder (mpeg2, mprg4 (DIVX 5), none)? [2/4/N] ");
  int k= getchar();
  switch(k) {
  case '2':
    sprintf(cmd,"for F in %s*.ppm; do echo \"convert $F\"; convert \"$F\" \"${F%%ppm}sgi\"; rm \"$F\"; done", filename); 
    printf("%s\n", cmd);
    system(cmd);    
    sprintf(cmd,"mencoder mf://%s*.sgi -mf w=%i:h=%i:fps=%i:type=sgi -ovc lavc -lavcopts vcodec=mpeg2video -oac copy -o %s.mpg", filename, w, h, 25, filename);
    printf("%s\n", cmd);
    system(cmd);
    break;
  case '4':
    sprintf(cmd,"for F in %s*.ppm; do echo \"convert $F\"; convert \"$F\" \"${F%%ppm}sgi\"; rm \"$F\"; done", filename); 
    printf("%s\n", cmd);
    system(cmd);    
    sprintf(cmd,"mencoder mf://%s*.sgi -mf w=%i:h=%i:fps=%i:type=sgi -ovc lavc -lavcopts vcodec=mpeg4 -oac copy -o %s.avi -ffourcc DX50", filename, w, h, 25, filename);
    printf("%s\n", cmd);
    system(cmd);
    break;
  default: 
    return;
  } 

  printf("Should I cleanup? [y/N] ");
  while((k= getchar()) <32);
  switch(k) {    
  case 'J':
  case 'j':
  case 'Y':
  case 'y':
    printf("Remove all temporary files\n");    
    sprintf(cmd,"rm -f %s*.sgi;", filename); 
    system(cmd);    
    break;    
  default: 
    break;        
  }
}

 
int main(int argc, char *argv[])
{
   if (argc!= 4)
   {
     fprintf(stderr,"Usage: %s file w h\n", argv[0]);
     exit(1);
   }
 
   char *fname = argv[1];
   int w = atoi(argv[2]);
   int h = atoi(argv[3]);
   assert(w>0);
   assert(h>0);
   
   FILE *f = fopen(fname,"rb");
   if (!f) 
   {
     fprintf(stderr,"Cannot open %s for reading\n", fname);
     exit(2);
   } 
   
   int framenr=0;
   int chunk = w*h*3;
   
   unsigned char *buf = new unsigned char [chunk];
 
 
   int retval;
   do
   {
     retval = fread(buf, chunk, 1, f);
     if (retval)
     {
       char outname[128];
       sprintf(outname, "%s%04d.ppm", fname, framenr);
       FILE *g=fopen(outname,"wb");
       if(!g) {
	 fprintf(stderr,"Cannot open file %s for writing", outname);
	 exit(1);	 
       }
       fprintf(g,"P6 %d %d 255\n", w, h);
       fwrite(buf, chunk, 1, g);
       fclose(g);
       framenr++;
     }
   } while (retval);
   printf("Wrote %d frames\n", framenr);
   mencoder(fname, w, h);
     

   return 0;
 }
 
