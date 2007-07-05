#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <iterator>

using namespace std;

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>

#include "serial_unix.h"
#include "cmdline.h"

#define C_R 0
#define C_D 1
#define C_S 2
#define C_M 3
#define C_E 4
#define C_V 5
#define C_A 7

/* The maximum number of NACKs to be send for a single data
   frame (adr, cmd, len [, data, len]). */
#define MAX_NACKS 2

#define READTIMEOUT 10
#define MAXFAILURES 10



typedef struct Xbee {
  Xbee(short addr) : addr(addr), initialised(false), failurecounter(0){}
  short addr;
  int numsensors;
  int sensoroffset;
  int nummotors;
  int motoroffset;
  bool initialised;
  int failurecounter;
}Xbee;

class Communicator;
ConfigList configs;
Communicator* communication;

void onTermination();

class Communicator : public AbstractRobot, public CSerialThread {
  enum State {PHASE_INIT, PHASE_CYCLE, PHASE_CYCLE_W4Ack};
  public:
  /** @param serial port name: e.g.: ttyS0
    @param baud rate e.g. 4800
    @param net self-defined xbee network number (master=net*16) e.g. 3
   */
    Communicator(const CString& port, int baud, 	       
                 AbstractController* controller, 
                 AbstractWiring* wiring,
                 const list<PlotOption>& plotOptions,
                 const vector<Xbee>& xbees,
                 int verboseMode,
                 bool test_mode =false)
  : AbstractRobot("RemoteRobot", "$Id$"), 
    CSerialThread(port,baud, test_mode), 
    controller(controller), wiring(wiring), plotOptions(plotOptions), xbees(xbees)
    { 
      sensornumber=0;
      motornumber=0; 
      numberinternals=0;
      x=0;
      y=0;
      noise=0.1;
      cycletime=50;
      cycletimechanged=false;
      pause=false;

      agent = 0;

      pendingTime=0;

      if (verboseMode==1)
        verbose=true;
      else if (verboseMode==2) {
        verbose=true;
        verboseMore=true;
      }
    }

    ~Communicator(){
      if(agent) delete agent;
      fprintf(stderr,"closed\n");
      if(x) free(x);
      if(y) free(y);
    }

  /**
     * This method creates an address packet 0000xxxx with xxxx indicating the slave
     * address, i.e. only the 4 lower bits are taken from the 'adr'.
   */
    virtual uint8 makeAddrPacket(uint8 adr) {
      return PADR | adr;
    }
  
    virtual uint8 makeStopPacket(uint8 adr) {
      return PSTOP | adr;
    }

  /**
     * This method creates an acknowledge packet 0001xxxx with xxxx indicating the
     * slave address.
   */
    virtual uint8 makeAckPacket(uint8 adr) {
      return PACK | adr;
    }

  /**
     * This method creates an not-acknowledge packet 0010xxxx with xxxx indicating the
     * slave address.
   */
    virtual uint8 makeNackPacket(uint8 adr) {
      return PNACK | adr;
    }

  /**
     * This method creates a command packet 01xxxxxx with xxxxxx indicating the command,
     * i.e. only the 6 lower bits are taken from the paramter cmd.
   */
    virtual uint8 makeCmdPacket(uint8 cmd) {
      return PCMD | cmd;
    }

  /**
     * This method creates a length packet 10xxxxxx with xxxxxx indicating the length,
     * i.e. only the 6 lower bits are taken from the paramter len.
     * Note: Since data bytes are splitted (see makeDataPackets()), the actual number
     * of data packets is twice the number of data bytes to be send. The length indicates
     * the number of data bytes, and not the number of data packets.
   */
    virtual uint8 makeLenPacket(uint8 len) {
      return PLEN | len;
    }

  /**
     * This method creates two data packets 11aaxxxx|11bbyyyy where xxxxyyyy is the original
     * data byte. Packets are numbered subsequently according to a mod 4 rule (00, 01, 10,
     * 11, 00, 01, ...; bits aa and bb, resp.).
   */
    virtual uint8* makeDataPackets(uint8 data, uint8* p, uint8 i) {
      p[0] = PDAT | (((2*i)   % 4) << 4) | ((data >> 4) & 15);
      p[1] = PDAT | (((2*i+1) % 4) << 4) | ( data       & 15);
      return p;
    }

    virtual void sendAck(uint8 adr) {
      sendByte(makeAckPacket(adr));
      return;
    }

    virtual void sendNack(uint8 adr) {
      sendByte(makeNackPacket(adr));
      return;
    }

  /**
     * This method writes len bytes of 'raw' data to the slave with the address 'adr'.
     * On success the net number of bytes (len) is returned, otherwise -1.
   */
    virtual int sendData(uint8 adr, uint8 cmd, uint8 *data, uint8 len) {
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
          cerr << "Got unexpected data (ACK/NACK expected).\n";
        }
        return -1;
      }

      return len;
    }

    virtual void receiveMsg(uint8 adr, int len) {
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
  
    virtual void printMsg(uint8* data, int len) {

      fprintf(stdout, "Message from slave: "); fflush(stdout);
      write(fileno(stdout), data, len);
      fprintf(stdout, "\n");
      return;
    }


  /**
     * This method writes a single byte to the serial output and returns the return
     * value of the write method.
   */
    virtual int sendByte(uint8 c) {
      return write(fd_out, &c, 1);
    }

    virtual int getByte(uint8 *c) {
      int cnt = 0, n;
      while ((n = read(fd_in, c, 1)) <= 0) {
        if (cnt++ > READTIMEOUT) {
          cerr << "Time out!\n";
          return -1;
        }
        usleep(10000);
      }
      return n;
    }

  /**
     * rn is the number of remaining NACKs to be send for this data frame.
   */
    virtual int receiveData(uint8 adr, uint8 *cmd, uint8 *data, uint8 maxlen, int rn) {
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
    
      /* Print out message. */
      if (*cmd == CDMSG) {
        printMsg(buffer, len);
        return receiveData(adr, cmd, data, maxlen, rn);
      }

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

    /// this function is calles at the beginning (initialised==false) or if connection is lost (initialised==true)
    virtual bool resetXbee(int currentXbee){
      if (sendData(xbees[currentXbee].addr, CBEEP, NULL, 0) < 0){
            cerr << "Error while sending beep.\n";
            return false;
          }
  
          /* Send 'reset' command to current slave. */
          if (sendData(xbees[currentXbee].addr, CRES, NULL, 0) < 0){
            cerr << "Error while sending reset.\n";
            return false;
          }
  
          /* Receive dimension, i.e. number of sensors/motors. */
          uint8 cmd;
          int len = receiveData(xbees[currentXbee].addr, &cmd, databuf, 2, MAX_NACKS);
          if ((cmd != CDIM) || (len != 2)) {
            cerr << "Didn't receive number of motors/sensors (len = " << len << ")\n";
            return false;
          }

          xbees[currentXbee].failurecounter=0;
          if(xbees[currentXbee].initialised == false){ // first reset
            xbees[currentXbee].numsensors   = databuf[0];
            xbees[currentXbee].sensoroffset = sensornumber_new;
            sensornumber_new += xbees[currentXbee].numsensors;
            xbees[currentXbee].nummotors    = databuf[1];
            xbees[currentXbee].motoroffset  = motornumber_new;
            motornumber_new += xbees[currentXbee].nummotors;

            xbees[currentXbee].initialised=true;  
            if(verbose){
              cout << "Dim for xbee " << currentXbee << " (adress: " << xbees[currentXbee].addr  << ") : "
                  << xbees[currentXbee].numsensors << ", "
                  << xbees[currentXbee].nummotors << endl;
              cout << "Offsets: " << xbees[currentXbee].sensoroffset << ", " 
                  << xbees[currentXbee].sensoroffset << endl;
            }
          }else{  // later resets -> lets check whether the same number of motors and sensors
             if(xbees[currentXbee].numsensors != databuf[0] || xbees[currentXbee].nummotors != databuf[1]){
               cerr << "Error: sensor or motor number not equivalent to initial values " << databuf[0] << ", " << databuf[1] << endl;
               exit(1);
             }
          }
          return true;
    }



    virtual void Initialise(){
      pendingTime=0;
      currentXbee=0;
      sensornumber_new=0;
      motornumber_new=0; 

      state=PHASE_INIT;

      if(verbose) {
        cout << "Initialise!\n";
      }

      // this might result in a random order of xbee initialisation -> random motor/sensor order
      bool allinitialised;
      do{
        allinitialised=true;
        for (currentXbee = 0; currentXbee < xbees.size(); currentXbee++) {
          if(xbees[currentXbee].initialised) continue;
          allinitialised &= resetXbee(currentXbee);
        }
        if(!allinitialised){
          cout << "Some Xbee was not responing, try again!\n";
          usleep(500000);
        }
      }while(!allinitialised);
      sensornumber = sensornumber_new;
      motornumber  = motornumber_new;
    
      initController();
    }

    virtual void printError() { //const DAT& s){
    //switch(s.buffer[0]){
    //case 0: cerr << "some error\n"; break;
    //default: cerr << " number " << s.buffer[0]; break;
    //}
    }

 
    virtual void writeMotors_readSensors() {
      int i, len, offset, n;

      for(currentXbee=0; currentXbee < xbees.size(); currentXbee++){
        if(xbees[currentXbee].failurecounter > MAXFAILURES){
          resetXbee(currentXbee);
          continue;
        }

        sendMotorCommands();
  
        /* Read sensor values. */
        offset = xbees[currentXbee].sensoroffset;
        n = xbees[currentXbee].numsensors;
        uint8 cmd;
        len = receiveData(xbees[currentXbee].addr, &cmd, databuf, n, MAX_NACKS);
  
        /* When data were successfully read. */
        if (len >= 0) {
  
          /* Print sensor values. */
          if (verbose) {
            fprintf(stdout, "Sensor values: ");
            for (i = 0; i < len; i++)
              fprintf(stdout, "%6i", (int) databuf[i]); //cout << (int) databuf[i] << ", ";
            fprintf(stdout, "\n"); //cout << "\n";
          }
  
          /* Check for command and number of data bytes. */
          if (cmd != CDSEN) {
            cerr << "Didn't receive sensor values (wrong command).\n";
            xbees[currentXbee].failurecounter++;
          }
  
          if (len != n) {
            cerr << "Wrong number of sensor values received (got " << len << ", " << n << " expected).\n";
            xbees[currentXbee].failurecounter++;
          }
  
          /* Write sensor values to the x vector. */
          for (i = 0; i < len; i++) {
            x[i+offset] = databuf[i] / 127.0 - 1.0;
          }
        }
        else{
          xbees[currentXbee].failurecounter++;
          if (verbose)
            cerr << "Did not receive sensor values.\n";
        }
      }
      /* Next step. */
      agent->step(noise);

      return;
    }

    virtual void loopCallback(){
      while(pause){
        usleep(1000);
      }
    }

    virtual void sendMotorCommands() {
      int i, len, offset, n;
      pendingTime=0;

      /* Send motor values. */
      offset = xbees[currentXbee].motoroffset;
      n      = xbees[currentXbee].nummotors;

      uint8 mdata[n];
      for (i = 0; i < n; i++) {
        mdata[i] = (uint8) (127.0 * (y[i + offset] + 1.0));
      }

      len = sendData(xbees[currentXbee].addr, CDMOT, mdata, n);
      if (verbose) {
        if (len == n) {
          fprintf(stdout, "Motor data: ");
          for (int i = 0; i < n; i++)
            fprintf(stdout, "%6i", (int) mdata[i]);
          fprintf(stdout, "   |   "); fflush(stdout);
        }
        else
          cerr << "Error while sending motor values.\n";
      }
      return;
    }

    virtual bool is_pending(int pendingTimeout) {
      if (pendingTime==pendingTimeout)
        return true;
      else
        pendingTime++;
      return false;
    }

  // robot interface

  /** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] 
    @param sensornumber length of the sensor array
    @return number of actually written sensors
   */
    virtual int getSensors(sensor* sensors, int sensornumber){
      assert(sensornumber == this->sensornumber);
      memcpy(sensors, x, sizeof(sensor) * sensornumber);
      return sensornumber;
    }

  /** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
   */
    virtual void setMotors(const motor* motors, int motornumber){
      assert(motornumber == this->motornumber);
      memcpy(y, motors, sizeof(motor) * motornumber);
    }

    /** returns number of sensors */
    virtual int getSensorNumber(){ return sensornumber; }

    /** returns number of motors */
    virtual int getMotorNumber() { return motornumber; }

    virtual Position getPosition() const {return Position(0,0,0);}
    virtual Position getSpeed() const {return Position(0,0,0);}
    virtual Position getAngularSpeed() const {return Position(0,0,0);}
    virtual matrix::Matrix getOrientation() const {
      matrix::Matrix m(3,3);
      m.toId();
      return m; 
    };

    virtual paramval getParam(const paramkey& key) const{
      if(key == "noise") return noise; 
      else if(key == "cycletime") return cycletime; 
      else if(key == "reset") return 0; 
      else  return Configurable::getParam(key);
    }

    virtual bool setParam(const paramkey& key, paramval val){
      if(key == "noise") noise = val; 
      else if(key == "cycletime"){
        cycletime=val;
        cycletimechanged=true;
      } else if(key == "reset"){
        Initialise();
      } else 
        return Configurable::setParam(key, val); 
        return true;
    }

    virtual paramlist getParamList() const {
      paramlist list;
      list += pair<paramkey, paramval> (string("noise"), noise);
      list += pair<paramkey, paramval> (string("cycletime"), cycletime);
      list += pair<paramkey, paramval> (string("reset"), 0);
      return list;
    };

  protected:
    void initController(){
      if(x) free(x);
      if(y) free(y);
      x=(double*)malloc(sizeof(double)*sensornumber);
      y=(double*)malloc(sizeof(double)*motornumber);

      for (int i = 0; i < motornumber; i++) {
        y[i] = 0.0;
      }

      agent = new Agent(plotOptions);
      agent->init(controller, this, wiring);
      cout << "# Initialised controller with " 
          << sensornumber << " Sensors and " 
          << motornumber  << " Motors\n";     
    }

  public:
    bool pause;


  private:
    int motornumber;
    int sensornumber;
    int motornumber_new;
    int sensornumber_new;
    int numberinternals;
    double noise;
    double cycletime;
    bool cycletimechanged;
    bool verbose;
    bool verboseMore;
    unsigned int currentXbee;
    State state;

    int pendingTime;

  //DAT motorDat;
    uint8 databuf[135];
  
    Agent* agent;
    AbstractController* controller;
    AbstractWiring* wiring;
    list<PlotOption> plotOptions;
    vector<Xbee> xbees;

    double* x;
    double* y;
}; 

void onTermination(){
  cmd_end_input();
  fprintf(stderr,"Try to stop serial thread\n");
  if(communication){    
    communication->stop();
  }
}

// Helper
int contains(char **list, int len,  const char *str){
  for(int i=0; i<len; i++){
    if(strcmp(list[i],str) == 0) return i+1;
  }
  return 0;
}


int main(int argc, char** argv){
  list<PlotOption> plotoptions;
  int verboseMode=0;
  const char* port = "/dev/ttyS0";

  vector<Xbee> xbees;
  xbees.push_back(Xbee(1));
  xbees.push_back(Xbee(2));

  AbstractController* controller = new InvertMotorSpace(10);
  //AbstractController* controller = new SineController();
  controller->setParam("s4delay",2.0);
  controller->setParam("s4avg",2.0);

  if(contains(argv,argc,"-g")!=0) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-s")!=0) plotoptions.push_back(PlotOption(SoundMan));
  if(contains(argv,argc,"-v")!=0) verboseMode=1;
  if(contains(argv,argc,"-vv")!=0) verboseMode=2;
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g] [-f] [-v] [-h] [-p port]\n",argv[0]);
    printf("\t-g\tstart guilogger\n\t-f\twrite logfile\n\t-h\tdisplay this help\n");
    printf("\t-v\tenable verbose mode\n\t-p port\tuse give serial port (/dev/ttyUSB0)\n");
    exit(0);
  }
  int index = contains(argv,argc,"-p");
  if(index && index<argc){
    port = argv[index];
    cout << "use port " << port << endl;
  }

  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");

  communication= new Communicator(port, 115200, controller, 
                                  new One2OneWiring(new ColorUniformNoise(0.01)),
                                  plotoptions, xbees, verboseMode);
  communication->start();
  cmd_handler_init();

  configs.push_back(communication);
  configs.push_back(controller);
  showParams(configs);

  while (communication->is_running()) {
    
    // check for cmdline interrupt
    if (control_c_pressed()) {
      communication->pause=true;
      changeParams(configs);
      cmd_end_input();
      communication->pause=false;
    }
  };

  fprintf(stderr,"Serial thread terminated\n");
  delete communication;
  delete controller;
  fprintf(stderr,"Cleaned up\n");

  return 0;
}
