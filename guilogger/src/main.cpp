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
#define DEBUG

#include <qapplication.h>
#include "guilogger.h"
#include "filelogger.h"
#include "qserialreader.h"
#include "qpipereader.h"

/**
  * \brief Main Programm
  * \author Dominic Schneider
  */
int main( int argc, char ** argv ) {
    QApplication a( argc, argv );

    QString mode;
    QDataSource *qsource;
    FileLogger fl;

    #ifdef DEBUG
       printf("DEBUG mode on!\n");
    #endif
    
    if(argc > 1) mode = argv[1];
    else printf("No mode selected.\n");

    if(mode=="serial")    
    {   printf("Using serial port as source.\n"); 
        qsource = new QSerialReader();
    }
    else if(mode=="pipe") qsource = new QPipeReader();
    else
    {    printf("Using default source: serial port\n");
         qsource = new QSerialReader();
         //((QSerialReader*) qsource)->setComport("/dev/ttyS1");
    }

    guilogger gl;

    a.connect(qsource, SIGNAL(newData(char *)), &gl, SLOT(receiveRawData(char *)));
    a.connect(qsource, SIGNAL(newData(char *)), &fl, SLOT(writeChannelData(char *)));  // the filelogger is listening
    qsource->start();

    gl.setCaption( "GUI Logger" );
    gl.show();

    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
    return a.exec();
}
