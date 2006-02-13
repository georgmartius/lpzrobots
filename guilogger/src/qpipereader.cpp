/***************************************************************************
 *   Copyright (C) 2005 by Dominic Schneider   *
 *   dominic@isyspc8   *
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
 ***************************************************************************/
#include "qpipereader.h"
#include <stdio.h>
#include <unistd.h>  //für usleep
#include <stdlib.h>

QPipeReader::QPipeReader(int delay)
{   
    this->delay = delay;// default parameter = 100
}

void QPipeReader::run()
{
  int size = 65536;
  char *s = (char*)malloc(size*sizeof(char));

  while(1) {
    char* ctrl;
    
    ctrl = fgets(s, size, stdin);
        
    emit newData(s);            
    //     usleep(delay);     // wenn Line gelesen, mal ein bissel warten, damit nicht alles gleich durchpfeift
  }
  free(s);  
}

