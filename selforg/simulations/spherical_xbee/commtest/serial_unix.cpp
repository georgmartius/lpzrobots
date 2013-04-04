#include "serial_unix.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <iostream>
#include <errno.h>


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

int CSerialThread::readB() {
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


int CSerialThread::readNByte(unsigned char* c, int n, int timeout){
  int r = read(fd_out, c, n);
  if(r<n){ // second try
    if(r<0) r=0;
    printf("Retry!\n");
    usleep(timeout);
    int r1= read(fd_out, c+r, n-r);
    if (r1<0)
      return -1;
    else
      r+=r1;
  }
  if(r==n){
    if(c[0]=='#' && c[1]=='#'){
      return r;
    }else{

    }
  }return -1;


}

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
  //  newtio.c_iflag = INPCK | IXON | IXOFF;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VMIN]=1;
  newtio.c_cc[VTIME]=0;

  tcsetattr(fd_in,TCSANOW,&newtio);
  tcflush(fd_in, TCIFLUSH);
  //    pthread_testcancel();

  unsigned char slave = 0;
  unsigned char c[40];
  int numslaves = 2;
  // main loop
  while(!terminated){
    pthread_testcancel();
    //!!test
    slave = (slave+1)%numslaves;
    printf("Send to %i\n",slave);
    c[0]=255;
    c[1]=slave+1;
    c[2]=17;
    write(fd_out, c, 20);

    int b;
    do{
      b = readB();
      if(b>31 && b<96) printf("%c",b);
    }while( (b != 255) && (b != -1));
    if (b == -1) continue;
    int addr = readB();
    if (addr == -1) continue;
    int len = readB();
    if (len == -1) continue;
    if(addr==0){
      int i;
      for(i=0; i<len; i++){
        int b =readB();
        if(b==-1) break;
        c[i] = b;
      }
      if(i<len) continue;
    }else{
      printf("Got weird packet\n");
    }

    // output
    printf("Got from %i: ",slave);
    for(int i=0; i<len; i++){
      printf("%i ",c[i]);
    }
    printf("\n");

  }//  end of while loop
  close(fd_in);
  fd_in=-1;
  if(test_mode) close(fd_out);
  m_is_running=false;
  return true;
};


/**
 * This method creates two data packets 11aaxxxx|11bbyyyy where xxxxyyyy is the original
 * data byte. Packets are numbered subsequently according to a mod 4 rule (00, 01, 10,
 * 11, 00, 01, ...; bits aa and bb, resp.).
 */
uint8* CSerialThread::makeDataPackets(uint8 data, uint8* p, uint8 i) {
  p[0] = PDAT | (((2*i)   % 4) << 4) | ((data >> 4) & 15);
  p[1] = PDAT | (((2*i+1) % 4) << 4) | ( data       & 15);
  return p;
}

void CSerialThread::sendAck(uint8 adr) {
  sendByte(makeAckPacket(adr));
  return;
}

void CSerialThread::sendNack(uint8 adr) {
  sendByte(makeNackPacket(adr));
  return;
}

/**
 * This method creates an address packet 0000xxxx with xxxx indicating the slave
 * address, i.e. only the 4 lower bits are taken from the 'adr'.
 */
uint8 CSerialThread::makeAddrPacket(uint8 adr) {
  return PADR | adr;
}

uint8 CSerialThread::makeStopPacket(uint8 adr) {
  return PSTOP | adr;
}

/**
 * This method creates an acknowledge packet 0001xxxx with xxxx indicating the
 * slave address.
 */
uint8 CSerialThread::makeAckPacket(uint8 adr) {
  return PACK | adr;
}

/**
 * This method creates an not-acknowledge packet 0010xxxx with xxxx indicating the
 * slave address.
 */
uint8 CSerialThread::makeNackPacket(uint8 adr) {
  return PNACK | adr;
}

/**
 * This method creates a command packet 01xxxxxx with xxxxxx indicating the command,
 * i.e. only the 6 lower bits are taken from the paramter cmd.
 */
uint8 CSerialThread::makeCmdPacket(uint8 cmd) {
  return PCMD | cmd;
}

/**
 * This method creates a length packet 10xxxxxx with xxxxxx indicating the length,
 * i.e. only the 6 lower bits are taken from the paramter len.
 * Note: Since data bytes are splitted (see makeDataPackets()), the actual number
 * of data packets is twice the number of data bytes to be send. The length indicates
 * the number of data bytes, and not the number of data packets.
 */
uint8 CSerialThread::makeLenPacket(uint8 len) {
  return PLEN | len;
}


/**
 * This method writes len bytes of 'raw' data to the slave with the address 'adr'.
 * On success the net number of bytes (len) is returned, otherwise -1.
 */
int CSerialThread::sendData(uint8 adr, uint8 cmd, uint8 *data, uint8 len) {
  int n;
  uint8 c;

  //uint8 buffer[size];
  if (sendByte(makeAddrPacket(adr)) <= 0) return -1; //buffer[0] = makeAddrPacket(adr);
  if (sendByte(makeCmdPacket(cmd))  <= 0) return -1;  //buffer[1] = makeCmdPacket(cmd);
  if (sendByte(makeLenPacket(len))  <= 0) return -1;  //buffer[2] = makeLenPacket(len);

  int i; uint8 d[2];
  for (i = 0; i < (int) len; i++) {
    makeDataPackets(data[i], d, i);
    if (sendByte(d[0]) <= 0) return -1;  //buffer[2*i + 3] = d[0];
    if (sendByte(d[1]) <= 0) return -1;  //buffer[2*i + 4] = d[1];
  }

  /* Indicate end of data stream with zero-length byte. */
  //    if (len > 0)
  //      if (sendByte(makeLenPacket(0)) <= 0) return -1; //buffer[size-1] = makeLenPacket(0);

  /* send stop byte: always when communication is finished */
  if (sendByte(makeStopPacket(adr)) <= 0) return -1; //buffer[size-1] = makeLenPacket(0);


  /* Check for NACK and eventually resend data. */
  n = getByte(&c);
  if ((n > 0) && (c == (PNACK | MSADR))) {
    if (verbose) cerr << "Got NACK: Resend data...\n";
    return sendData(adr, cmd, data, len);
  }
  if (n <= 0) {
    if (verbose) cerr << "Did not get ACK/NACK.\n";
    return -1;
  }
  if (c != (PACK  | MSADR)) {
    if (verbose) {
      cerr << "Got unexpected data (ACK/NACK expected): |";
      printf(" %i\n",c);
    }
    return -1;
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

int CSerialThread::getByte(uint8 *c) {
  int cnt = 0, n=-1;

  while ((n = read(fd_in, c, 1)) <= 0) {
    if (cnt++ > READTIMEOUT) {
      cerr << "Time out!\n";
      return -1;
    }
    usleep(1000);
  }
//   if (errno==EAGAIN)
//     cerr << "EAGAIN detected!" << endl;
//   cout << "errno: " << errno << endl;
  return n;
}

void CSerialThread::receiveMsg(uint8 adr, int len) {
  int n;
  uint8 c, msg[len];

  /* Read data packets. */
    for (int i = 0; i < len; i++) {

      /* Get first data packet (i.e. MS bits from data byte). */
      n = getByte(&c); //n = read(fd_in, &c, 1);
      if ((n < 1) || ((c & MASK_L2) != PDAT))
        return;
      msg[i] = (MASK_R4 & c) << 4;

      /* Get second data packet (i.e. LS bits from data byte). */
      n = getByte(&c);
      if ((n < 1) || ((c & MASK_L2) != PDAT))
        return;
      msg[i] = msg[i] | (MASK_R4 & c);

      //if ((i % 2) == 1) sendAck(adr);
    }

    /* If any data packets were read: get final zero-length packet. */
    //    if (len > 0) {
    n = getByte(&c);
    if ((n != 1) || (c != (PSTOP | MSADR))) {
      sendNack(adr);
      return;
    }
    sendAck(adr);
    //  }

    fprintf(stdout, "Message from slave: "); fflush(stdout);
    write(fileno(stdout), msg, len);
    fprintf(stdout, "\n");
    return;
}


/**
 * rn is the number of remaining NACKs to be send for this data frame.
 */
int CSerialThread::receiveData(uint8 adr, uint8 *cmd, uint8 *data, uint8 maxlen, int rn) {
  int n, len = 0;
  uint8 c;
  uint8 buffer[128];

  /* Check whether the maximum number of trials is reached. */
  if (rn <= 0) return -1;

  /* Read packets until address packet with own address has been read. */
  int cnt = 0;
  do {
    n = getByte(&c);
    if (cnt++ > 0) return -1;
  } while ((n < 1) || (c != (PADR | MSADR)));

  /* Read and check for command byte. */
  n = getByte(&c); //read(fd_in, &c, 1);
  if ((n < 1) || ((c & MASK_L2) != PCMD)) {
    cerr << "Command packet expected but not received!\n";
    sendNack(adr);
    return receiveData(adr, cmd, data, maxlen, rn-1);
  }
  *cmd = c & ~MASK_L2;

  /* Read and check for length byte. */
  n = getByte(&c); //read(fd_in, &c, 1);
  if ((n < 1) || ((c & MASK_L2) != PLEN)) {
    cerr << "Length packet expected but not received!\n";
    sendNack(adr);
    return receiveData(adr, cmd, data, maxlen, rn-1);
  }
  len = c & ~MASK_L2;

  //sendAck(adr);


  /* Read data packets. */
  for (int i = 0; i < len; i++) {
    if (i >= 128) return -1;
    /*if (i > maxlen) {
      cerr << "Data buffer size of " << (int) maxlen << " is too small" <<
      " (received " << len << " bytes).\n";
      return -1;
      }*/

    /* Get first data packet (i.e. MS bits from data byte). */
    n = getByte(&c); //read(fd_in, &c, 1);
    if ((n < 1) || ((c & MASK_L2) != PDAT)) {
      cerr << "Error while receiving data from slave.\n";
      sendNack(adr);
      return receiveData(adr, cmd, data, maxlen, rn-1); }
    buffer[i] = (MASK_R4 & c) << 4;

    /* Get second byte packet (i.e. LS bits from data byte). */
    n = getByte(&c); //read(fd_in, &c, 1);
    if ((n < 1) || ((c & MASK_L2) != PDAT)) {
      cerr << "Error while receiving data from slave.\n";
      sendNack(adr);
      return receiveData(adr, cmd, data, maxlen, rn-1); }
    buffer[i] = buffer[i] | (MASK_R4 & c);

    //if ((i % 2) == 1) sendAck(adr);
  }

  /* If any data packets were read: get final zero-length packet. */
  //    if (len > 0) {
  n = getByte(&c); //read(fd_in, &c, 1);
  if ((n < 1) || (c != (PSTOP | MSADR))) {
    cerr << "Data format error: did not receive final zero-length packet: n= " << n << "\n";
    sendNack(adr);
    return receiveData(adr, cmd, data, maxlen, rn-1);
  }
  //  }
  sendAck(adr);

//   /* Print out message. */
//   if (*cmd == CDMSG) {
//     printMsg(buffer, len);
//     return receiveData(adr, cmd, data, maxlen, rn);
//   }

  if (len > maxlen) {
    cerr << "Data buffer size of " << (int) maxlen << " is too small" <<
      " (received " << len << " bytes).\n";
    return -1;
  }

  /* Copy buffer to data. */
  for (int i = 0; i < len; i++)
    data[i] = buffer[i];

  return len;
}



/// redirection function, because we can't call member function direct
static void* CSerialThread_run(void* p)
{

  bool rv = ((CSerialThread*)p)->run();
  pthread_exit(&rv);

}

