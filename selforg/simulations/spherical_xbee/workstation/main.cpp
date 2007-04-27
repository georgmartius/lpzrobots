// 'Protokoll Binary 
// ' Format: CLXXXXX ...
// '  C=command byte 0cccaaaa 
// '    ccc Command:  000 R = Reset, 001 D = Dimension, 010 = Sensors
// '                  011 M = Motors 
// '                  100 E = Error, 101 V = Verbose
// '    aaaa Address: 4 subnets - from 0 to 3 and 4 to 7...
// '             0,4,8,12  master(s) (PC)
// '             rest indiviual addresses
// '            }
// '  L=length byte 1lllllll
// '    lllllll: Length of message
// '  X=value byte

// ' Communication (< from Robot, > from Workstation)
// ' > R1 0
// ' < D0 2 NumSensors NumMotors
// ' > M1 2 0 0   ' initial motor values 0
// ' < S0 3 s1 s2 s3  
// ' > M1 2 m1 m2 
// ' < S0 3 s1 s2 s3
// ' .....
// ' > M1 2 m1 m2 
// ' < E0 1 1

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

typedef struct Xbee {
  Xbee(short addr) : addr(addr), initialised(0){}
  short addr;
  int numsensors;
  int sensoroffset;
  int nummotors;
  int motoroffset;
  bool initialised;
}Xbee;

class Communicator;
ConfigList configs;
Communicator* communication;

void onTermination();

class Communicator : public AbstractRobot, public CSerialThread {
  enum State {PHASE_INIT, PHASE_CYCLE};
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
               bool verbose,
	       bool test_mode =false)
    : AbstractRobot("RemoteRobot", "$Id$"), 
      CSerialThread(port,baud, test_mode), verbose(verbose),
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

    agent = 0;
  }

  ~Communicator(){
    if(agent) delete agent;
    fprintf(stderr,"closed\n");
    if(x) free(x);
    if(y) free(y);
  }

  virtual void Initialise(){
    currentXbee=0;
    sensornumber_new=0;
    motornumber_new=0; 
    DAT d(C_R,xbees[currentXbee].addr,0); //send reset to first xbee
    state=PHASE_INIT;
    d.send(fd_out);
  }

  virtual void ReceivedCommand(const DAT& s){
    if(verbose) {
      cout << "CMD: ";
      s.print();
    }
      
    short cmd = s.buffer[0] >> 4;   // select upper 4 bit
    short addr = s.buffer[0] & 0x0F; // select last 4 bit
    short len = s.buffer[1] & 0x7F; // select last 7 bit
    if(verbose) cout << "Packet: CMD: " << cmd << " ADDR: " << addr << " LEN " << len << endl;
    int readlen;
    DAT c;
    if(addr & 0x03 !=0){ // not a packet for master
      cout << "# not a packet for master\n";
      if(len>0){
	readlen=0;
	do{ 
	  loopCallback();
	  readlen += read(fd_in, c.buffer+readlen, 1);
	}while(readlen<len);
      }
      return;
    }else{ // our packet 
      if(len>0){
	readlen=0;
	do{ 
	  loopCallback();
	  readlen += read(fd_in, c.buffer+readlen, 1);
	}while(readlen<len);
	c.len=len;
      }
      ProcessCmd(cmd,c);
    }
  }

  virtual void printError(const DAT& s){
    switch(s.buffer[0]){
    case 0: cerr << "some error\n"; break;
    default: cerr << " number " << s.buffer[0]; break;
    }
  }

  virtual void ProcessCmd(short cmd, const DAT& s){
    if(verbose) {
      cout << "DATA:\n";
      s.print();
    }
    switch(cmd){
    case C_D:
      if(state!=PHASE_INIT) { cerr << "got unexpected dimension\n"; return;}      
      if(s.len!=2) { cerr << "wrong D packet"; return;}
      xbees[currentXbee].numsensors   = s.buffer[0];
      xbees[currentXbee].sensoroffset = sensornumber_new;
      sensornumber_new += xbees[currentXbee].numsensors;
      xbees[currentXbee].nummotors=s.buffer[1];
      xbees[currentXbee].motoroffset = motornumber_new;
      motornumber_new += xbees[currentXbee].nummotors;
      xbees[currentXbee].initialised=true;
      if(verbose){
	cout << "Dim for xbee " << currentXbee << ": " 
	     << xbees[currentXbee].numsensors << ", "
	     << xbees[currentXbee].nummotors << endl;
	cout << "Offets: " << xbees[currentXbee].sensoroffset << ", " 
	     << xbees[currentXbee].sensoroffset << endl;	
      }
      currentXbee++;
      break;
    case C_S:
      if(state!=PHASE_CYCLE){  cerr << "got unexpected sensors\n"; return;}
      if(sensornumber>0 && x){ 
	if(s.len != xbees[currentXbee].numsensors){	   
	  cerr << "wrong number of sensors received: " << s.len 
	       << " expected " << xbees[currentXbee].numsensors << endl;
        }
	int offset = xbees[currentXbee].sensoroffset;
	if(verbose) cout << "Sensors: "; 
	for(int i = 0; i < s.len; i++){	  	  
	  x[i+offset] = (s.buffer[i]/ 127.0)-1.0;
	  if(verbose) cout << x[i+offset] << " " ;
	}
	if(verbose) cout << endl;
      }else{cerr << "Initialisation error\n";}
      break;
    case C_E:
      cerr << "ERROR:"; 
      if(s.len<0) printError(s); 
      break;
    case C_V:
      cerr << "MSG:"; 
      if(s.len<0) s.print(); 
      break;
    default:
      break;
    }
    
    switch (state){
    case PHASE_INIT:
      if(currentXbee>=xbees.size()){
	state=PHASE_CYCLE;
	currentXbee=0;
	if(sensornumber_new==sensornumber && motornumber_new == motornumber){
	  cout << "# No changes in motor and sensornumber. Keep controller!\n"; 
	}else{
	  if(sensornumber_new<1 || motornumber_new<1) {
	    cerr << "# Sensor or Motor number 0! "; 
	  } else {
	    sensornumber = sensornumber_new;
	    motornumber  = motornumber_new;
	    initController();
	  }
	}
      }else{
	DAT d(C_R,xbees[currentXbee].addr,0); //send reset to next xbee
	d.send(fd_out);
      }
      break;
    case PHASE_CYCLE:
      if(currentXbee>=xbees.size()){
	if (verbose) cout << "Step " << endl; 
	// calls controller and asks us about sensors and stores motors
	agent->step(noise);
	currentXbee=0;
      }
      // send motor values
      DAT d(C_M,xbees[currentXbee].addr,xbees[currentXbee].nummotors); 
      int offset = xbees[currentXbee].motoroffset;
      if(verbose) cout << "Motors to " << currentXbee << ": "; 
      for(int i=0; i < xbees[currentXbee].nummotors; i++){
	d.buffer[i+2] = (unsigned char)((y[i+offset]+1.0)*127.0);
	if(verbose) printf("%i ",d.buffer[i+2]);
      }
      if(verbose) cout << endl;
      d.send(fd_out);      
      break;
    }
  }

  virtual void loopCallback(){
    // check for cmdline interrupt
    if (control_c_pressed()){
      changeParams(configs);
      cmd_end_input();
    }
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
    else  return Configurable::getParam(key);
  }

  virtual bool setParam(const paramkey& key, paramval val){
    if(key == "noise") noise = val; 
    else if(key == "cycletime"){
      cycletime=val;
      cycletimechanged=true;
    } else if(key == "reset"){
      initController();
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

    agent = new Agent(plotOptions);
    agent->init(controller, this, wiring);
    cout << "# Initialised controller with " 
	 << sensornumber << " Sensors and " 
	 << motornumber  << " Motors\n";     
  }

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
  unsigned int currentXbee;
  State state;
  
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

void test(){
  FILE* f = fopen("protokoll_test", "wb");
  unsigned char d[10];
  d[0] = (C_D << 4);
  d[1] = 128+2;
  d[2] = 4;
  d[3] = 2;
  fwrite(d,1,4,f);
  d[0] = C_S << 4;
  d[1] = 128+4;
  d[2] = 128;
  d[3] = 48;
  d[4] = 49;
  d[5] = 255;
  fwrite(d,1,6,f);
  fclose(f);
}

int main(int argc, char** argv){
  list<PlotOption> plotoptions;
  bool verbose = false;

  //  test();  return 1;

  vector<Xbee> xbees;
  xbees.push_back(Xbee(1));
  //  xbees.push_back(Xbee(2));

  //  AbstractController* controller = new InvertMotorSpace(10);
  AbstractController* controller = new SineController();
  controller->setParam("s4delay",2.0);
  controller->setParam("s4avg",2.0);

  if(contains(argv,argc,"-g")!=0) plotoptions.push_back(PlotOption(GuiLogger));
  if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
  if(contains(argv,argc,"-v")!=0) verbose=true;
  if(contains(argv,argc,"-h")!=0) {
    printf("Usage: %s [-g] [-f]\n",argv[0]);
    printf("\t-g\tstart guilogger\n\t-f\twrite logfile\n\t-h\tdisplay this help\n");
    exit(0);
  }

  printf("\nPress Ctrl-c to invoke parameter input shell (and again Ctrl-c to quit)\n");
  
  communication= new Communicator("protokoll_test", 4800, controller, 
//  communication= new Communicator("/dev/ttyUSB0", 4800, controller, 
				  new One2OneWiring(new ColorUniformNoise(0.01)),
				  plotoptions, 
				  xbees,
				  verbose, true);
  communication->start();
  cmd_handler_init();

  configs.push_back(communication);
  configs.push_back(controller);
  showParams(configs);

  while(communication->is_running()){
    usleep(1000);
  };  



  fprintf(stderr,"Serial thread terminated\n");
  delete communication;
  delete controller;
  fprintf(stderr,"Cleaned up\n");


  return 0;
}


