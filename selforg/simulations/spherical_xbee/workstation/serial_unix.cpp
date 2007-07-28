#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <iostream>
#include <errno.h>
#include "serial_unix.h"


static void* CSerialThread_run(void* p);

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
  terminated=true;
  pthread_testcancel();
};


// thread function
bool CSerialThread::run(){
  
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
  if (fd_in <0) { cerr << "Error open port.\n"; return false; }
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
 
  tcsetattr(fd_in,TCSANOW,&newtio);
  tcflush(fd_in, TCIFLUSH);
  //    pthread_testcancel();

  Initialise();

  // main loop
  while(!terminated){
    pthread_testcancel();
    /* Get sensor values / send motor values. */
    writeMotors_readSensors();
    loopCallback();
  }//  end of while loop
  close(fd_in);
  fd_in=-1;
  if(test_mode) close(fd_out);
  m_is_running=false;
  return true;
};


/**
 * This method writes len bytes of 'raw' data to the slave with the address 'adr'.
 * On success the net number of bytes (len) is returned, otherwise -1.
 */
int CSerialThread::sendData(uint8 adr, uint8 cmd, uint8 *data, uint8 len) {  
  if (sendByte(255) <= 0) return -1;   // Marker
  if (sendByte(adr) <= 0) return -1;  
  if (sendByte(cmd) <= 0) return -1;  
  if (sendByte(len) <= 0) return -1;  

  int i; 
  for (i = 0; i < (int) len; i++) {
    if(data[i]==255) data[i]=254;
    if (sendByte(data[i]) <= 0) return -1;
  }      
  return len;
}


/**
 * This method writes a single byte to the serial output and returns the return
 * value of the write method.
 */
int CSerialThread::sendByte(uint8 c) {
  return write(fd_out, &c, 1);
}

int CSerialThread::getByte() {
  int cnt = 0, n=-1;
  unsigned char c;
  while ((n = read(fd_in, &c, 1)) <= 0) {
    if (cnt++ > READTIMEOUT) {
      cerr << "Time out!\n";
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

/**
 * returns the number received bytes
 */
int CSerialThread::receiveData(uint8 my_adr, uint8 *cmd, uint8 *data) {
  int b;
  do{
    b = getByte();    
  }while( (b != 255) && (b != -1));
  if (b == -1) return -1;
  int addr = getByte();
  if (addr == -1) return -1;
  int command = getByte();
  if (command == -1) return -1;
  *cmd = command;
  int len = getByte();
  if (len == -1) return -1;
  if(addr==my_adr){
    int i;
    for(i=0; i<len; i++){
      int b =getByte();
      if(b==-1) return -1; 
      data[i] = b;
    }
  }else{
    printf("Got weird packet\n");    
  }
  return len;
}



/// redirection function, because we can't call member function direct
static void* CSerialThread_run(void* p)
{
  
  bool rv = ((CSerialThread*)p)->run();
  pthread_exit(&rv);

}

