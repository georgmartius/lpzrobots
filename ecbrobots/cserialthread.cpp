/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    marcoeckert@web.de                                                   *
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
 *   Revision 1.5  2009-03-25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.4  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.3  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include "cserialthread.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <iostream>
#include <errno.h>

namespace lpzrobots {


static void* CSerialThread_run(void* p);


CSerialThread::CSerialThread(const CString& port, int baud, int serialReadTimeout, bool debug, bool test_mode) : debug(debug), test_mode(test_mode), m_port(port),m_baud(baud),  terminated(false),  serialReadTimeout(serialReadTimeout), m_is_joined(true){};


void CSerialThread::setConfig(const CString& port, int baud, int serialReadTimeout, bool debug, bool test_mode) {
	this->debug=debug;
	this->test_mode=test_mode;
	this->m_port=port;
	this->m_baud=baud;
	this->terminated=false;
	this->serialReadTimeout=serialReadTimeout;
	this->m_is_joined=true;
  }

/// start serial communication
void CSerialThread::start(){
  m_is_running=true;
  m_is_joined=false;
  terminated=false;
  fd_in=-1;
  fd_out=-1;
  // start thread using this static function

  pthread_create(&thread,NULL,CSerialThread_run, this);

};

/// stop serial communication
void CSerialThread::stopandwait(){
  if(!m_is_joined)
    {
      // set stop signal
      terminated=true;
      usleep(1000);
      pthread_testcancel();
      usleep(1000);
      //      pthread_cancel(thread);
      pthread_join(thread,NULL);
      m_is_joined=true;
      m_is_running=false;
    }
};

/// stop serial communication
void CSerialThread::stop(){
  if (debug)
    std::cout << "CSerialThread: stop." << std::endl;
  terminated=true;
  pthread_testcancel();
};


// thread function
bool CSerialThread::run(){

  if (!internInit())
    stop();

  // init function of the ECBCommunicator
  initialise();

  if (debug)
    std::cout << "CSerialThread: finished initialising. Starting the loop." << std::endl;
  /* main loop which calls the loop
  * function of the ecbcommunicator
  */
  bool inLoop=true;

  while(!terminated && inLoop){
    pthread_testcancel();
    inLoop = loop();
  }
  if (debug)
    std::cout << "CSerialThread: End of loop reached. Closing streams..." << std::endl;

  // close stream and others...
  close(fd_in);
  fd_in=-1;
  if(test_mode) close(fd_out);
  m_is_running=false;
  return true;
};


/**
 * This method writes a single byte to the serial output
 * @return true if the byte was sent, otherwise false
 */
bool CSerialThread::sendByte(uint8 c) {
  //cout << c << " " << endl;
  
  if (write(fd_out, &c, 1)==1) {
    if (debug) printf("[%3d]", c) ;
    return true;
  }
  if (debug) printf("--");
  return false;
}

int CSerialThread::getByte() {
  int cnt = 0, n=-1;
  unsigned char c;
  while ((n = read(fd_in, &c, 1)) <= 0) {
    if (cnt++ > serialReadTimeout) {
      return -1;
    }
    usleep(1000);
  }
  return c;
}


void CSerialThread::flushInputBuffer(int wait) {
  char buffer [128];
  usleep(wait*1000);
  read(fd_in, buffer, 128);
}

bool CSerialThread::internInit() {
  if (debug)
    std::cout << "CSerialThread: internal initialising..." << std::endl;
  int baud;
  struct termios newtio;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,0);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,0);

  switch(m_baud){
  case   1200: baud=B1200;   break;
  case   2400: baud=B2400;   break;
  case   4800: baud=B4800;   break;
  case   9600: baud=B9600;   break;
  case  19200: baud=B19200;  break;
  case  38400: baud=B38400;  break;
  case  57600: baud=B57600;  break;
  case 115200: baud=B115200; break;
  default:
    return false;
  }

  // open port, non-block for error handling is necessary
  fd_in = open(m_port.c_str(), O_RDWR|O_SYNC|O_NONBLOCK);
  //fd_in = open(m_port.c_str(), O_RDWR);
  //    pthread_testcancel();
  if (fd_in <0) { cerr << "CSerialThread: Error open port.\n"; return false; }
  if(test_mode){
    fd_out = open((m_port + "_out").c_str(), O_RDWR|O_SYNC);//|O_NONBLOCK);
  }else{
    fd_out=fd_in;
  }

  // set interface parameters
  newtio.c_cflag = baud | CS8 | CLOCAL | CREAD | CSTOPB;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VMIN]=1;
  newtio.c_cc[VTIME]=0;
  //newtio.c_cc[VEOL]=10; //Enter-key is EOL

  tcsetattr(fd_in,TCSANOW,&newtio);
  tcflush(fd_in, TCIFLUSH);
  //    pthread_testcancel();
  if (debug)
    std::cout << "CSerialThread: Opened port " << m_port << " with " << m_baud << " Baud." << std::endl;
  return true;
}

/// redirection function, because we can't call member function direct
static void* CSerialThread_run(void* p)
{
  bool rv=false;
  CSerialThread* ct = dynamic_cast<CSerialThread*>((CSerialThread*)p);
  if(ct)
    rv = ct->run();
  else{
    cerr << "scheisse" << endl;
  }
  pthread_exit(&rv);
}

}
