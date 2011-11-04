/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Dominic Schneider (original author)                                  *
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
#include "qpipereader.h"
#include <stdio.h>
#include <unistd.h>  //for usleep
#include <stdlib.h>

QPipeReader::QPipeReader(int delay, FILE* f) {
  this->f = f;
  this->delay = delay;// default parameter = 0
}

void QPipeReader::run() {
  // one data element uses in the #C line (name) normally between 7 and 16 signs
  // so at 16 signs per element the number of elements is limited to 4096, which is not
  // sufficient for big and many matrices.
  // allow up to 64MB usage of memory (*1024) instead of only 64KB, hence 4M elements are allowed (matrix: 2Kx2K)
  int size = 65536 * 1024;
  char *s = (char*) malloc(size * sizeof(char));

  char* ctrl = s;
  while (ctrl) {
    
    ctrl = fgets(s, size, f);
    if (ctrl) {
      emit newData(QString(s));
    }
    if (delay)
      usleep(delay);
  }
  free(s);
}

