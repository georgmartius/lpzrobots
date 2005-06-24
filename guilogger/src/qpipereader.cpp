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

QPipeReader::QPipeReader(char bt, int delay)
{   blockterminator = bt;    // default parameter = 100
    this->delay = delay;
//    setbuf(stdin, NULL);  // nützt auch nix
}

void QPipeReader::run()
{
    char *s = NULL;
    int size = 0;

    while(1)
    {
        char c;
        int i=0;

        do{
//            i = fscanf(stdin, "%c", &c);
            i = fread(&c, 1, 1, stdin);
            if(i==0) usleep(delay);  // mal ein bissel schlafen wenn nix da ist um die Last zu senken
        } while(i!=1);

        size++;
        s = (char*) realloc( s, size);
        s[size-1] = c;

        if(c==blockterminator)   // check if we got a line ending
        {   // s now contains a complete line readed from serial port
            s[size-1] = '\0';       // make s a zero terminated string (ZTS)
            emit newData(s);
            printf("Readed: %s\n", s);
            
            free(s);
            s=NULL;
            size=0;
            usleep(delay);     // wenn Line gelesen, mal ein bissel warten, damit nicht alles gleich durchpfeift
        }
    }

}
