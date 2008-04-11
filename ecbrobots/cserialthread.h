#ifndef __CSERIALTHREAD_H_
#define _CSERIALTHREAD_H_

#include <pthread.h>
#include <string>

#include "globaldata.h"

using namespace std;

namespace lpzrobots {

typedef string CString;



/** Thread-Klasse, die in eigenem Thread die Kommunikation mit der seriellen Schnittstelle uebernimmt. */
class CSerialThread {

protected:
  int fd_in; // file handle for incomming
  int fd_out; // file handle for outgoing (=fd_in expect in file test mode)
  bool debug;
  bool test_mode;

public:

  CSerialThread(const CString& port, int baud, int serialReadTimeout, bool debug=false, bool test_mode=false);

  virtual ~CSerialThread(){stopandwait();};

     /// thread function
  bool run();

  /// thread is running?
  bool is_running(){return m_is_running;};

    /// start serial communication
  void start();
  /// stop serial communication and wait for the thread to terminate
  void stopandwait();
  /// stop serial communication (call also be called from inside)
  virtual void stop();


protected:

  virtual bool loop() = 0;


  virtual bool sendByte(uint8 c);

  virtual int getByte();

  /// is called at the beginning after initialisation of serial
  virtual void initialise() = 0;

  virtual void flushInputBuffer(int wait);


private:

  CString m_port;
  int m_baud;
  bool terminated;
  int serialReadTimeout;
  bool m_is_joined;
  bool m_is_running;


  pthread_t thread;


  bool internInit();

};

}

#endif
