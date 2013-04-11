#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <pthread.h>
#include <string>
using namespace std;

typedef string CString;

#define PADR    0x0  /* 00000000 */
#define PACK    0x10 /* 00010000 */
#define PNACK   0x20 /* 00100000 */
#define PSTOP   0x30 /* 00110000 */
#define PCMD    0x40 /* 01000000 */
#define PLEN    0x80 /* 10000000 */
#define PDAT    0xc0 /* 11000000 */

#define CRES    0x1  /* 00000001 Reset command                            */
#define CDIM    0x2  /* 00000010 Dimension data: number of sensors/motors */
#define CDSEN   0x3  /* 00000011 Sensor data values                       */
#define CDMOT   0x4  /* 00000100 Motor data values                        */
#define CBEEP   0x8  /* 00001000 Make a beep    */
#define CDMSG   0x9  /* 00001001 Message data.  */

#define MASK_L2 0xc0 /* 11000000 */
#define MASK_L4 0xf0 /* 11110000 */
#define MASK_R4 0xf  /* 00001111 */

#define MSADR   0x0  /* Master address */

/* The maximum number of NACKs to be send for a single data
   frame (adr, cmd, len [, data, len]). */
#define MAX_NACKS 2

#define READTIMEOUT 30
#define MAXFAILURES 4

typedef unsigned char uint8;


// pretend to have some windows data types
typedef unsigned long DWORD;
typedef unsigned int UINT;

/** Thread-Klasse, die in eigenem Thread die Kommunikation mit der seriellen Schnittstelle uebernimmt. */
class CSerialThread{
  CString m_port;
  int m_baud;
  bool m_is_running;
  bool terminated;
  bool m_is_joined;

  pthread_t thread;
  bool test_mode;

protected:
  int fd_in; // file handle for incomming
  int fd_out; // file handle for outgoing (=fd_in expect in file test mode)

  bool verbose;
  bool verboseMore;
public:

  CSerialThread(const CString& port, int baud, bool test_mode=false)
    : m_port(port),m_baud(baud), terminated(false), m_is_joined(true), test_mode(test_mode){};
  virtual ~CSerialThread(){stopandwait();};

  virtual int sendByte(uint8 c);
  virtual int getByte(uint8 *c);
  virtual int receiveData(uint8 adr, uint8 *cmd, uint8 *data, uint8 maxlen, int rn);
  virtual void receiveMsg(uint8 adr, int len);

  /**
   * This method creates two data packets 11aaxxxx|11bbyyyy where xxxxyyyy is the original
   * data byte. Packets are numbered subsequently according to a mod 4 rule (00, 01, 10,
   * 11, 00, 01, ...; bits aa and bb, resp.).
   */
  virtual uint8* makeDataPackets(uint8 data, uint8* p, uint8 i);
  /**
   * This method creates an address packet 0000xxxx with xxxx indicating the slave
   * address, i.e. only the 4 lower bits are taken from the 'adr'.
   */
  virtual uint8 makeAddrPacket(uint8 adr);

  virtual uint8 makeStopPacket(uint8 adr);

  /**
   * This method creates an acknowledge packet 0001xxxx with xxxx indicating the
   * slave address.
   */
  virtual uint8 makeAckPacket(uint8 adr);

  /**
   * This method creates an not-acknowledge packet 0010xxxx with xxxx indicating the
   * slave address.
   */
  virtual uint8 makeNackPacket(uint8 adr);

  /**
   * This method creates a command packet 01xxxxxx with xxxxxx indicating the command,
   * i.e. only the 6 lower bits are taken from the paramter cmd.
   */
  virtual uint8 makeCmdPacket(uint8 cmd);

  /**
   * This method creates a length packet 10xxxxxx with xxxxxx indicating the length,
   * i.e. only the 6 lower bits are taken from the paramter len.
   * Note: Since data bytes are splitted (see makeDataPackets()), the actual number
   * of data packets is twice the number of data bytes to be send. The length indicates
   * the number of data bytes, and not the number of data packets.
   */
  virtual uint8 makeLenPacket(uint8 len);

  /**
   * This method writes len bytes of 'raw' data to the slave with the address 'adr'.
   * On success the net number of bytes (len) is returned, otherwise -1.
   */
  virtual int sendData(uint8 adr, uint8 cmd, uint8 *data, uint8 len);
  virtual void sendAck(uint8 adr);
  virtual void sendNack(uint8 adr);


  /// thread is running?
  bool is_running(){return m_is_running;};

  /// start serial communication
  void start();
  /// stop serial communication and wait for the thread to terminate
  void stopandwait();
  /// stop serial communication (call also be called from inside)
  void stop();

  /// set com port
  void comport(const CString& port){ m_port=port; };
  /// set baud rate
  void baudrate(int baud){ m_baud=baud; };


  int readB();
  int readNByte(unsigned char*, int n, int timeout);

  /// thread function
  bool run();
};

#endif
