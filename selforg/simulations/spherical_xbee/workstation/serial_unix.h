#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <pthread.h>
#include <string>
using namespace std;

typedef string CString;
typedef struct DAT {
  DAT(){ len = 0;}
  DAT(const DAT& d) {len=d.len, memcpy(buffer,d.buffer,len);}
  DAT(short len): len(len) {}
  DAT(short cmd, short addr, short datalen): len(datalen+2) {
    buffer[0]= (cmd<<4) + addr;
    buffer[1]= datalen | (1 << 7);
  }
  void print() const;
  bool send(int fd);
  
  unsigned char buffer[128];
  short len;
} DAT;

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
 protected:
    int fd; // file handle

 public:

    CSerialThread(const CString& port, int baud)
	: m_port(port),m_baud(baud), terminated(false), m_is_joined(true){};
    virtual ~CSerialThread(){stopandwait();};

    // for communication with inherited class
    
    /// (log) received string 
    virtual void ReceivedCommand(const DAT& s) = 0;
    /// is called in every loop
    virtual void loopCallback() = 0;
    /// is called at the beginning after initialisation of serial
    virtual void Initialise() = 0;
    /// reads a line terminated by eol from serial
    CString CSerialThread::readline();


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

    /// thread function
    bool run();
};

#endif
