#include "serial_unix.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>


static void* CSerialThread_run(void* p);

void DAT::print() const {
  printf("A: ");
  for( short i=0; i<len; i++){
    if(buffer[i] >32 && buffer[i] < 128)
      printf("%c",buffer[i]);
    else 
      printf(" ");
  }
  printf("\nH: ");
  for( short i=0; i<len; i++){
    printf("%02X ",buffer[i]);
  }
  printf("\n");
}

bool DAT::send(int fd){
  int written = write(fd,buffer,len);
  return (written==len);
}

/// start serial communication
void CSerialThread::start(){
  m_is_running=true;
  m_is_joined=false;
  terminated=false;
  fd=-1;
  // start thread using this static function
  pthread_create(&thread,NULL,CSerialThread_run, this);
};

/// stop serial communication
void CSerialThread::stopandwait(){
  if(!m_is_joined)
    {
      // set stop signal
      terminated=true;
      pthread_testcancel();
      //      pthread_cancel(thread);
      pthread_join(thread,NULL);
      m_is_joined=true;
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
  case 1200:baud=B1200;break;
  case 2400:baud=B2400;break;
  case 4800:baud=B4800;break;
  case 9600:baud=B9600;break;
  case 19200:baud=B19200;break;
  case 38400:baud=B38400;break;
  case 57600:baud=B57600;break;
  default: 
    return false;
  }

  // open port
  fd = open(m_port.c_str(), O_RDWR|O_SYNC);//|O_NONBLOCK);
  //    pthread_testcancel();
  if (fd <0) return false;
  
  // set interface parameters
  newtio.c_cflag = baud | CS8 | CLOCAL | CREAD | CSTOPB;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VMIN]=1;
  newtio.c_cc[VTIME]=0;
 
  tcsetattr(fd,TCSANOW,&newtio);
  //    pthread_testcancel();
  tcflush(fd, TCIFLUSH);
  //    pthread_testcancel();

  DAT s(2);
  
  Initialise();

  // main loop
  while(!terminated){
    int i;
    pthread_testcancel();
    do{
      loopCallback();
      i=read(fd,s.buffer,2);
      pthread_testcancel();
    } while(i!=2);
    
    ReceivedCommand(s);        
//     // check if we got a line ending
//     if(c=='\n')
//       {
// 	// process comand
// 	ReceivedCommand(s);
// 	ans=ParseCommandLine(s);
// 	if(ans.size()!=0){
// 	  SentAnswer(ans);

// 	  pthread_testcancel();
// 	  // send answer
// 	  write(fd,ans.c_str(), ans.size());
// 	}
// 	s.clear();
//       }
  }//  end of while loop
  close(fd);
  fd=-1;
  m_is_running=false;
  return true;
};

CString CSerialThread::readline(){
  int i=0;
  char c;
  CString s="";
  do{
    do{
      loopCallback();
      i=read(fd,&c,1);
      pthread_testcancel();
    } while(i!=1);
    s+=c;
    // check if we got a line ending
  } while(c!='\n');
  return s;
}


/// redirection function, because we can't call member function direct
static void* CSerialThread_run(void* p)
{
  
  bool rv = ((CSerialThread*)p)->run();
  pthread_exit(&rv);

}

