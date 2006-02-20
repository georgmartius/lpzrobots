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

#include <signal.h>

#include <qapplication.h>
#include "guilogger.h"
#include "filelogger.h"
#include "qserialreader.h"
#include "qpipereader.h"
#include "commlineparser.h"


int Control_C;
void signal_handler_exit(void){
  signal(SIGINT,SIG_DFL);
}

void control_c(int ){
  Control_C++ ;
  if (Control_C > 100) exit(0);
}


/**
 * We need to catch Ctrl-C (SIGINT) because if we are called from another program (like ode simulation) that reacts on Ctrl-C we are killed. 
 * SIGPIPE is emitted if the stdin or stdout breaks. 
 * We need to terminate if the stdin breaks to get closed with the calling application (in  pipe mode).
*/
void signal_handler_init(){
  signal(SIGINT,control_c);
  atexit(signal_handler_exit);
  Control_C=0;
  signal(SIGPIPE, SIG_DFL);
}

/**
  * \brief Main Programm
  * \author Dominic Schneider
  */
int main( int argc, char ** argv ) {   
    signal_handler_init();

   CommLineParser params;
   params.parseCommandLine(argc, argv);

   if(params.getHelp())
   {   printf("guilogger parameter listing\n");
       printf("   -m [mode]  mode = serial | pipe | file\n");
       printf("   -p [port]  port = serial port to read from\n");
       printf("   -f [file]  input file\n");
       printf("      only viwewing, no streaming\n");
       printf("   -l turns logging on\n");
       printf("   -d [delay] delay for piped input, should be a natural number\n");
       printf("   --help displays this message.\n");
       return 0;
   }

    QApplication a( argc, argv );

    QDataSource *qsource=0;


    if(params.getMode().isEmpty() && !params.getFile().isEmpty()) {
      params.setMode("file");
    }

    guilogger *gl = new guilogger(params);

    if(params.getMode()=="serial")    
    {   QSerialReader *qserial = new QSerialReader();
        if(params.getPort() != "") qserial->setComPort(params.getPort());
        printf("Using serial port %s as source.\n", qserial->getComPort().latin1());
        qsource = qserial;
        a.connect(qsource, SIGNAL(newData(char *)), gl, SLOT(receiveRawData(char *)));
        qsource->start();
    }
    else if(params.getMode()=="pipe") {  
      QPipeReader *qpipe = new QPipeReader();
      //        if(params.getDelay() >= 0) qpipe->setDelay(params.getDelay());
      //        printf("Using pipe input with delay %i.\n", qpipe->getDelay());
      printf("Using pipe input\n");
      qsource = qpipe;
      a.connect(qsource, SIGNAL(newData(char *)), gl, SLOT(receiveRawData(char *)));
      qsource->start();
    } else if(params.getMode()=="file") 
    {  // printf("Sorry, there are no native segfaults any more.\n");
//        printf("But nevertheless I further provide segfaults for convenience by using free(0)\n");
//        printf("Just kidding! Have a nice day.\n");
    } else {
      fprintf(stderr, "Specify mode (-m) or file (-f) and use (-h) for help.\n");
      exit(1);
    }
   

    FileLogger fl;
    if(params.getLogg()) 
    {   fl.setLogging(TRUE);
        printf("Logging is on\n");
        a.connect(qsource, SIGNAL(newData(char *)), &fl, SLOT(writeChannelData(char *)));  // the filelogger is listening
    }
    else printf("Logging is off\n");

//    if(params.getMode() != "file") qsource->start();

    gl->setCaption( "GUI Logger" );
    gl->show();

    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
    return a.exec();
}
