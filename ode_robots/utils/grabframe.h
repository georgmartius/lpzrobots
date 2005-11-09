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
