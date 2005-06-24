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

class CommLineParser
{
private:
    QString mode;    // input streaming mode = serial | pipe | file
    QString port;    // serial port to read from
    QString file;    // input file for visualisation 
    bool    logg;    // Logging on/off
    bool    help;    // display help or not
    int     delay;   // delay for pipe
    
public:
    
    CommLineParser()
    {   logg = FALSE;
        help = FALSE;
        delay = 100;
    }
/*
    void setMode(QString mode) {this->mode = mode;}
    void setPort(QString port) {this->port = port;}
    void setFile(QString file) {this->file = file;}
    void setLogg(QString logg) {this->logg = logg;}
*/
    QString getMode()  {return mode;}
    QString getPort()  {return port;}
    QString getFile()  {return file;}
    bool    getLogg()  {return logg;}
    bool    getHelp()  {return help;}
    int     getDelay() {return delay;}
    
    void parseCommandLine(int argc, char **argv)
    {   QValueList<QString> ComLineParams;
        for(int i=1; i<argc; i++) ComLineParams.push_back(argv[i]);
    
        QValueList<QString>::iterator it;
        if((it=ComLineParams.find("-m")) != ComLineParams.end()) mode = *(++it);
        if((it=ComLineParams.find("-p")) != ComLineParams.end()) port = *(++it);
        if((it=ComLineParams.find("-f")) != ComLineParams.end()) file = *(++it);
        if((it=ComLineParams.find("-d")) != ComLineParams.end()) delay = (*(++it)).toInt();
        if(    ComLineParams.find("-l")  != ComLineParams.end()) logg = TRUE;
        if(    ComLineParams.find("--help")  != ComLineParams.end()) help = TRUE;

    }
};


int Control_C;
void signal_handler_exit(void){
  signal(SIGINT,SIG_DFL);
}

void control_c(int i){
  Control_C++ ;
  if (Control_C > 100) exit(0);
}

void signal_handler_init(){
  signal(SIGINT,control_c);
  atexit(signal_handler_exit);
  Control_C=0;  
}

/**
  * \brief Main Programm
  * \author Dominic Schneider
  */
int main( int argc, char ** argv ) {
   signal_handler_init();
  
   CommLineParser config;
   config.parseCommandLine(argc, argv);
  
   QApplication a( argc, argv );
    
   QString mode;
   QDataSource *qsource;
   
    
  
    guilogger gl;
/*
    printf("Mode %s\n", config.getMode().latin1());
    printf("Port %s\n", config.getPort().latin1());
    printf("File %s\n", config.getFile().latin1());
    if(config.getLogg()) printf("Logging\n");
    if(config.getHelp()) printf("Help\n");
*/
        
    
    
    
    
    if(config.getMode()=="serial")    
    {   QSerialReader *qserial = new QSerialReader();
        if(config.getPort() != "") qserial->setComPort(config.getPort());
        printf("Using serial port %s as source.", qserial->getComPort().latin1());
        qsource = qserial;
    }
    else if(config.getMode()=="pipe") 
    {   QPipeReader *qpipe = new QPipeReader();
        if(config.getDelay() >= 0) qpipe->setDelay(config.getDelay());
        printf("Using pipe input with delay %i.\n", qpipe->getDelay());
        qsource = qpipe;
    }
    else if(config.getMode()=="file") 
    {   printf("Sorry, not yet implemented.\n");
        printf("Hope you are lucky with a segfault.\n");
        printf("To produce more segfaults just try again\n");
        printf("Have a nice day.\n");
    }
    else
    {    QSerialReader *qserial = new QSerialReader();
         if(config.getPort() != "") qserial->setComPort(config.getPort());
         printf("Using serial communication as default on port %s\n", qserial->getComPort().latin1());
         qsource = qserial;
    }

    if(config.getLogg())
    {   FileLogger fl;
        a.connect(qsource, SIGNAL(newData(char *)), &fl, SLOT(writeChannelData(char *)));  // the filelogger is listening
    }

    a.connect(qsource, SIGNAL(newData(char *)), &gl, SLOT(receiveRawData(char *)));

    if(config.getHelp())
    {   printf("guilogger parameter listing\n");
        printf("   -m [mode]  mode = serial | pipe | file\n");
        printf("   -p [port]  port = serial port to read from\n");
        printf("   -f [file]  input file\n");
        printf("      only viwewing, no streaming\n");
        printf("   -l turns logging on\n");
        printf("   -d [delay] delay should be a natural number\n");
        printf("   --help Displays this message.\n");
    }
    
    
    qsource->start();

    gl.setCaption( "GUI Logger" );
    gl.show();

    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
    return a.exec();
}
