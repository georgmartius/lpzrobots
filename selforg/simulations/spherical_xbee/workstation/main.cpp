#include <signal.h>
#include <sys/time.h>
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
#include <selforg/dercontroller.h>

#include "serial_unix.h"
#include "console.h"
#include "cmdline.h"
#include "globaldata.h"

#define MAXFAILURES 4
#define MSADR   0x0  /* Master address */

#define CRES    0x1  /* 00000001 Reset command                            */
#define CDIM    0x2  /* 00000010 Dimension data: number of sensors/motors */
#define CSEN   0x3  /* 00000011 Sensor data values                       */
#define CMOT   0x4  /* 00000100 Motor data values                        */
#define CBEEP   0x8  /* 00001000 Make a beep    */
#define CMSG    0x9  /* 00001001 Message data.  */
#define CTEST  0x10  /* 00001010 Test i2c communication.  */


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
Communicator* communication;

void onTermination();

/**
   This class is the robot and the communicator in one.
   This might be a bit confusing, because it contains also the agent and so on.
   The main flow is as follows:
     loop:
      over serial we send motor values to the Xbees
      we wait for sensor values from Xbees
      then we invoke the agent
      the agent asks the robot (us) about sensors (which we saved in a local variable)
      the agent calls the controller and then sets the motor values with setMotors (also in this class)
     endloop
*/
class Communicator : public AbstractRobot, public CSerialThread {
  enum State {PHASE_INIT, PHASE_CYCLE};
public:
  /** @param serial port name: e.g.: ttyS0
      @param baud rate e.g. 38400
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
    x=0;
    y=0;
    noise=0.1;
    cycletime=50;
    pause=false;
    doReset=false;
    agent = new Agent(plotOptions);

    if (verboseMode==1)
      verbose=true;
    else if (verboseMode==2) {
      verbose=true;
      verboseMore=true;
    }
    resetSyncTimer();
  }

  ~Communicator(){
    if(agent) delete agent;
    fprintf(stderr,"closed\n");
    if(x) free(x);
    if(y) free(y);
  }

  virtual void printMsg(int xbee, int addr, uint8* data, int len) {
    data[len]=0;
    fprintf(stdout, "Message from slave (%i (addr: %i)): %s\n", xbee, addr, data);
  }

  /** this function is called at the beginning (initialised==false)
      or if connection is lost (initialised==true)
  */
  virtual bool resetXbee(int currentXbee){
    //    flush input buffer
    flushInputBuffer(cycletime/2);

    /*if (sendData(xbees[currentXbee].addr, CBEEP, NULL, 0) < 0){
      cerr << "Error while sending beep.\n";
      return false;
    }*/

   /* Send 'CTEST' command to current slave, which causes the xbee to test if i2c communication is ok! */
    if (sendData(xbees[currentXbee].addr, CTEST, NULL, 0) < 0){
      cerr << "Error while sending test command.\n";
      return false;
    }

    /* Send 'reset' command to current slave. */
    if (sendData(xbees[currentXbee].addr, CRES, NULL, 0) < 0){
      cerr << "Error while sending reset.\n";
      return false;
    }

    /* Receive dimension, i.e. number of sensors/motors. */
    uint8 cmd;
    int len;
    do{
      len = receiveData(MSADR, &cmd, databuf);
      if(cmd== CMSG) printMsg(currentXbee, xbees[currentXbee].addr, databuf,len);
    }while(cmd== CMSG);

    if ((cmd != CDIM) || (len != 2)) {
      cerr << "Didn't receive number of motors/sensors (len = " << len << ")\n";
      return false;
    }

    xbees[currentXbee].failurecounter=0;
    if(xbees[currentXbee].initialised == false){ // first reset

      cout << "Offset (sensor, motor) for " << currentXbee << " is (" << sensornumber_new << ", " << motornumber_new << ")\n";

      xbees[currentXbee].numsensors   = databuf[0];
      xbees[currentXbee].sensoroffset = sensornumber_new;
      sensornumber_new += xbees[currentXbee].numsensors;
      xbees[currentXbee].nummotors    = databuf[1];
      xbees[currentXbee].motoroffset  = motornumber_new;
      motornumber_new += xbees[currentXbee].nummotors;

      xbees[currentXbee].initialised=true;
      if(verbose){
        cout << "Dim for xbee " << currentXbee << " (address: " << xbees[currentXbee].addr  << ") : "
             << xbees[currentXbee].numsensors << ", "
             << xbees[currentXbee].nummotors << endl;
        cout << "Offsets: " << xbees[currentXbee].sensoroffset << ", "
             << xbees[currentXbee].motoroffset << endl;
      }
    }
    else{  // later resets -> lets check whether the same number of motors and sensors
      if(xbees[currentXbee].numsensors != databuf[0] || xbees[currentXbee].nummotors != databuf[1]){
        cerr << "Error: sensor or motor number not equivalent to initial values " << databuf[0] << ", " << databuf[1] << endl;
        // TODO: save controller and //exit(1);
      }
    }
    return true;
  }



  virtual void Initialise(){
    currentXbee=0;
    sensornumber_new=0;
    motornumber_new=0;

    state=PHASE_INIT;

    if(verbose) {
      cout << "Initialise!\n";
    }

//     // this might result in a random order of xbee initialisation -> random motor/sensor order
//     bool allinitialised;
//     do{
//       allinitialised=true;
//       for (currentXbee = 0; currentXbee < xbees.size(); currentXbee++) {
//         if(xbees[currentXbee].initialised) continue;
//         allinitialised &= resetXbee(currentXbee);
//       }
//       if(!allinitialised){
//         cout << "Some Xbee was not responing, try again!\n";
//         usleep(500000);
//       }
//     }while(!allinitialised);

    // New sequencial initialisation
    for (currentXbee = 0; currentXbee < xbees.size(); currentXbee++) {
      while(!resetXbee(currentXbee)){
        if (verbose) cout << "Xbee " << currentXbee << " not responing, try again!\n";
        usleep(5000);
      };

    }

    sensornumber = sensornumber_new;
    motornumber  = motornumber_new;

    cout << "Motors:  " << motornumber  << endl;
    cout << "Sensors: " << sensornumber << endl;

    initController();
  }


  Agent* getAgent(){
    return agent;
  }

  virtual void writeMotors_readSensors() {
    int i, len, offset, n;

    for(currentXbee=0; currentXbee < xbees.size(); currentXbee++) {
      if(doReset || xbees[currentXbee].failurecounter > MAXFAILURES){
        resetXbee(currentXbee);
      }
    }
    doReset=false;

    /* Print motor values. */
    if (verbose) {
      fprintf(stdout, "Motor data   : ");
      for (int i = 0; i < motornumber; i++)
        fprintf(stdout, "%6i", (int) (127.0 * (y[i] + 1.0)));
      fprintf(stdout, "  |  "); fflush(stdout);
    }

    for(currentXbee=0; currentXbee < xbees.size(); currentXbee++) {
      /* Send motor commands for currentXbee. */
      offset = xbees[currentXbee].motoroffset;
      n      = xbees[currentXbee].nummotors;

      for (i = 0; i < n; i++) {
        databuf[i] = (uint8) (127.0 * (y[i + offset] + 1.0));
      }

      len = sendData(xbees[currentXbee].addr, CMOT, databuf, n);
      if (len != n){
        cerr << "Error while sending motor values.\n";
        continue;
      }

      /* Read sensor values. */
      offset = xbees[currentXbee].sensoroffset;
      n = xbees[currentXbee].numsensors;
      uint8 cmd;
      do{
        len = receiveData(MSADR, &cmd, databuf);
        if(cmd== CMSG) printMsg(currentXbee, xbees[currentXbee].addr, databuf,len);
      }while(cmd== CMSG || cmd == CDIM);

      /* When data were successfully read. */
      if (len >= 0) {

        /* Check for command and number of data bytes. */
        if (cmd != CSEN) {
          cerr << "Didn't receive sensor values (wrong command : " << (int)cmd  << ".\n";
          xbees[currentXbee].failurecounter++;
          continue;
        }

        if (len != n) {
          cerr << "Wrong number of sensor values received (got " << len << ", " << n << " expected).\n";
          xbees[currentXbee].failurecounter++;
          continue;
        }

        /* Write sensor values to the x vector. */
        for (i = 0; i < len; i++)
          x[i+offset] = databuf[i] / 127.0 - 1.0;
      } else{
        xbees[currentXbee].failurecounter++;
        if (verbose)
          cerr << "Did not receive sensor values.\n";
        continue;
      }
      xbees[currentXbee].failurecounter = max(0, xbees[currentXbee].failurecounter-1);
    }

    /* Print sensor values. */
    if (verbose) {
      fprintf(stdout, "Sensor values: ");
      for (int i = 0; i < sensornumber; i++)
        fprintf(stdout, "%6i", (int) (127.0 * (x[i] + 1.0)));
      fprintf(stdout, "\n");
    }

    /* Next step. */
    agent->step(noise);

    return;
  }

  long timeOfDayinMS(){
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec*1000 + t.tv_usec/1000;
  }

  void resetSyncTimer(){
    realtimeoffset = timeOfDayinMS();
  }

  virtual void loopCallback(){
    /************************** Time Syncronisation ***********************/
    // Time syncronisation of real time and simulations time (not if on capture mode, or pause)
    if(!pause){
      long elaped = timeOfDayinMS() - realtimeoffset;
      // difference between actual passed time and desired cycletime
      long diff = cycletime - elaped;
      if(diff > 10000 || diff < -10000)  // check for overflow or other weird things
        resetSyncTimer();
      else{
        if(diff > 4){ // if less the 3 milliseconds we don't call usleep since it needs time
          usleep((diff-2)*1000);
        }else if (diff < 0){
          if (verbose) {
            printf("Time leak of %li ms detected\n",abs(diff));
          }
        }
      }
    }else {
      while(pause){
        usleep(1000);
      }
    }
    resetSyncTimer();
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

  virtual paramval getParam(const paramkey& key, bool traverseChildren) const{
    if(key == "noise") return noise;
    else if(key == "cycletime") return cycletime;
    else if(key == "reset") return 0;
    else  return Configurable::getParam(key);
  }

  virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren){
    if(key == "noise") noise = val;
    else if(key == "cycletime"){
      cycletime=(long)val;
    } else if(key == "reset"){
      doReset=true;
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
  double noise;
  long cycletime;
  unsigned int currentXbee;
  State state;
  long realtimeoffset;
  bool doReset;


  //DAT motorDat;
  uint8 databuf[255];

  Agent* agent;
  AbstractController* controller;
  AbstractWiring* wiring;
  list<PlotOption> plotOptions;
  vector<Xbee> xbees;

  double* x;
  double* y;
};

/// special wiring for the spherical Robot
class OurWiring : public AbstractWiring{
public:
  OurWiring(NoiseGenerator* noise, const std::string& name = "OurWiring")
    : AbstractWiring(noise, Controller, name){
    lastmotors = 0;
    numMotorSensors=2;
  }
  virtual ~OurWiring() {
    if(lastmotors) free(lastmotors);
  }

  virtual bool init(int robotsensornumber, int robotmotornumber){
    rsensornumber = robotsensornumber;
    rmotornumber  = robotmotornumber;
    // csensornumber = rsensornumber;
    csensornumber = numMotorSensors + (rsensornumber-numMotorSensors)*2;    // for IR sensors create two sensors each
    cmotornumber  = rmotornumber;
    if(!noiseGenerator) return false;
    noiseGenerator->init(csensornumber); // initialised with number of sensors ( see add() below)
    lastmotors = (motor*)malloc(sizeof(motor)* rmotornumber);
    return true;
  }

  virtual bool wireSensors(const sensor* rsensors, int rsensornumber,
                           sensor* csensors, int csensornumber,
                           double noise){
    // copy motor sensors
    for(int i=0; i< numMotorSensors; i++){
      csensors[i] = rsensors[i];
    }
    // for IR Sensors
    int j = numMotorSensors;
    for(int i=numMotorSensors; i< rsensornumber; i++){
      csensors[j]   = rsensors[i] * lastmotors[0];
      csensors[j+1] = rsensors[i] * lastmotors[1];
      j+=2;
    }
    return true;
  }

  virtual bool wireMotors(motor* rmotors, int rmotornumber,
                                         const motor* cmotors, int cmotornumber){
    memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
    memcpy(lastmotors, cmotors, sizeof(motor)*rmotornumber);
    return true;
  }


private:
  motor* lastmotors;
  int numMotorSensors;
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
  int baud = 38400;
  GlobalData globaldata;
  initializeConsole();

  vector<Xbee> xbees;
 xbees.push_back(Xbee(1));
//  xbees.push_back(Xbee(2));

   DerControllerConf dcc = DerController::getDefaultConf();
 //  AbstractController* controller = new DerController(dcc);
  AbstractController* controller = new SineController();
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
    printf("\t-b baud\tset baud rate\n");
    printf("\t-v\tenable verbose mode\n\t-p port\tuse give serial port (/dev/ttyUSB0)\n");
    exit(0);
  }
  int index = contains(argv,argc,"-p");
  if(index && index<argc){
    port = argv[index];
    cout << "use port " << port << endl;
  }
  index = contains(argv,argc,"-b");
  if(index && index<argc){
    baud = atoi(argv[index]);
    cout << "use baud rate " << baud << endl;
  }

  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");

  communication= new Communicator(port, baud, controller,
                                  // new OurWiring(new ColorUniformNoise(0.01)),
                                  new One2OneWiring(new ColorUniformNoise(0.01)),
                                  plotoptions, xbees, verboseMode);

  globaldata.configs.push_back(communication);
  globaldata.configs.push_back(controller);
  globaldata.agents.push_back(communication->getAgent());

  showParams(globaldata.configs);

  communication->start();
  cmd_handler_init();
  int counter = 0;
  while (communication->is_running()) {

    // check for cmdline interrupt
    if (control_c_pressed()) {
      communication->pause=true;
      usleep(200000);
      if(!handleConsole(globaldata)){
        communication->stopandwait();
      }
      cmd_end_input();
      communication->pause=false;
    }
    counter++;
  };

  fprintf(stderr,"Serial thread terminated\n");
  delete communication;
  delete controller;
  closeConsole();
  fprintf(stderr,"Cleaned up\n");

  return 0;
}
