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
#include "qserialreader.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>


QSerialReader::QSerialReader( char bt)
{    port="/dev/ttyS0";
     baudrate=19200;
     blockterminator = bt;
}


void QSerialReader::run()
{
    int fd=-1; // file handle
    int baud;
    struct termios newtio;

    switch(baudrate){
        case 1200:baud=B1200;break;
        case 2400:baud=B2400;break;
        case 9600:baud=B9600;break;
        case 19200:baud=B19200;break;
        case 38400:baud=B38400;break;
        case 57600:baud=B57600;break;
        default: 
            return;
    }

    fd = open(port, O_RDWR |O_SYNC);//|O_NONBLOCK);  // open port

    if (fd <0) 
    {   printf("Cannot open serial port %s\n", port.latin1()); 
        return;
    }
    // set interface parameters
    newtio.c_cflag = baud | CS8 | CLOCAL | CREAD | CSTOPB;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN]=1;
    newtio.c_cc[VTIME]=0;

    tcsetattr(fd,TCSANOW,&newtio);
    tcflush(fd, TCIFLUSH);

    char *s = NULL;
    int size = 0;

    while(1)    // main loop
    {   char c;
        int i;

        do{
            i=read( fd, &c, 1);        //  get one character from port fd
        } while(i!=1);

        if(size > 0 && c=='#') size=0;  // neue Channel Zeile fängt mitten drinne irgendwie an

        size++;
        s = (char*) realloc( s, size);
        s[size-1] = c;

        if(c==blockterminator || c==13 || c==10)  // check if we got a line ending
        {   // now s contains a complete line readed from serial port
            s[size-1] = '\0';       // makes s a Zero terminated string (ZTS)

            if(size > 3)
            {   // printf("Readed: %s\n", s);
                emit newData(s);
            }

            free(s);
            s = NULL;
            size=0;
        }

    }   //  end of while loop
    close(fd);
    fd=-1;
}
