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
 *   Revision 1.4  2008-04-08 10:11:03  guettler
 *   alpha testing
 *
 *   Revision 1.3  2008/04/08 09:12:34  martius
 *   *** empty log message ***
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include "ecbcommunicator.h"

#include <selforg/agent.h>

#include <sys/time.h>

#include "ecbagent.h"

using namespace std;

ECBCommunicator::ECBCommunicator(GlobalData& globalData):CSerialThread(globalData.portName, globalData.baudrate, globalData.serialReadTimeout, globalData.debug), globalData(&globalData) {
  if (this->globalData->debug)
    std::cout << "New ECBCommunicator created." << std::endl;
  realtimeoffset = timeOfDayinMS();
  lastBenchmarkTime = realtimeoffset;
  this->globalData->comm=this;
}

void ECBCommunicator::flushInputBuffer(int time) {
  if (globalData->debug)
    std::cout << "ECBCommunicator: flushInputBuffer!" << std::endl;
  if (time==-1)
    time = globalData->cycleTime/2;
  CSerialThread::flushInputBuffer(time);
}

  bool ECBCommunicator::loop() {
    assert (globalData->comm==this);
    globalData->simStep++;
    if (globalData->debug)
      std::cout << "ECBCommunicator: loop! simStep=" << globalData->simStep << std::endl;
    /// With this for loop all agents perform a controller
    /// AND a robot step, this means, the communication with the
    /// hardware ECBs are startet too!
      FOREACH (AgentList,globalData->agents,a) {
        ((ECBAgent*)(*a))->step(globalData->noise);
      }
    // sorgt dafür, dass der Zeittakt eingehalten wird:
    // Berechnung zu schnell -> warte,
    // Berechnung zu langsam -> Ausgabe, dass time leak stattfindet
      loopCallback();
    return true;
  }

  long ECBCommunicator::timeOfDayinMS(){
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec*1000 + t.tv_usec/1000;
  }

  void ECBCommunicator::resetSyncTimer(){
    realtimeoffset = timeOfDayinMS();
  }

  void ECBCommunicator::loopCallback(){
    /************************** Time Syncronisation ***********************/
    // Time syncronisation of real time and simulations time (not if on capture mode, or pause)
    if(!globalData->pause){
      long currentTime = timeOfDayinMS();
      int benchmarkSteps = 10;
      long elapsed = currentTime - realtimeoffset;
      if (globalData->benchmarkMode || globalData->debug) {
        std::cout << "Elapsed time: " << elapsed << "ms" << std::endl;
        if (globalData->benchmarkMode && (globalData->simStep % benchmarkSteps ==0)) {
          std::cout
            << "Benchmark: "
            << (((double)benchmarkSteps)/((double)(currentTime -lastBenchmarkTime)) *1000.0)
            << " cycles/s"
            << std::endl;
          lastBenchmarkTime = currentTime;
        }
      }
      // difference between actual passed time and desired cycletime
      long diff = globalData->cycleTime - elapsed;
      if(diff > 10000 || diff < -10000)  // check for overflow or other weird things
	resetSyncTimer();
      else{
	if(diff > 4){ // if less the 3 milliseconds we don't call usleep since it needs time
	  usleep((diff-2)*1000);
	}else if (diff < 0){
	  if (globalData->verbose) {
	    printf("Time leak of %li ms detected\n",abs(diff));
	  }
	}
      }
    }else {
      while(globalData->pause){
	usleep(1000);
      }
    }
    resetSyncTimer();
  }

  /**
   * PROTOCOL (without XBee handling!):
   * 1. send start byte
   * 2. send source address
   * 3. send destination address
   * 4. send command byte
   * 5. send data length byte
   * 6. send data bytes
   * 7. send crc byte (optional)
   */
  bool ECBCommunicator::sendData(commData& datac) {
    if (globalData->debug) cout << "Sending data: ";
    // start byte
    if (!(sendByte(255) && sendByte(globalData->masterAddress) && sendByte(datac.destinationAddress)
          && sendByte(datac.command) && sendByte(datac.dataLength))) {
      if (globalData->debug) cout << endl;
      return false;
    }
    for (int i=0;i<datac.dataLength;i++) {
        if (!sendByte(datac.data[i])) {
          if (globalData->debug) cout << endl;
          return false;
        }
      }
    // insert crc byte here!
    if (globalData->debug) cout << endl;
    return true;
  }


  /**
   * PROTOCOL (without XBee handling!):
   * 1. wait for receiving start byte
   * 2. receive source address
   * 3. receive destination address
   * 4. receive command byte
   * 5. receive data length byte
   * 6. receive data bytes
   * 7. receive crc byte (optional)
   *
   * @return the data that was received
   */
commData ECBCommunicator::receiveData() {
  if (globalData->debug) cout << "Receiving data: ";
  commData result;
  result.commSuccess=false;
  result.destinationAddress=255;
  result.sourceAddress=255;
  result.command=0;
  result.dataLength=0;

  // wait for start byte
  int b;
  do{
    b = getByte();
    if (globalData->debug) cout << b << " ";
  }while( (b != 255) && (b != -1));
  if (b == -1) {
    if (globalData->debug) cout << endl;
    return result;
  }

  // sourceAddress
  b = getByte();
  if (b == -1)  {
    if (globalData->debug) cout << endl;
    return result;
  }
  if (globalData->debug) cout << b << " ";
  result.sourceAddress=b;

  // destinationAddress
  b = getByte();
  if (b == -1)  {
    if (globalData->debug) cout << endl;
    return result;
  }
  if (globalData->debug) cout << b << " ";
  result.destinationAddress=b;

  // command
  b = getByte();
  if (b == -1)  {
    if (globalData->debug) cout << endl;
    return result;
  }
  if (globalData->debug) cout << b << " ";
  result.command=b;

  // dataLength
  b = getByte();
  if (b == -1)  {
    if (globalData->debug) cout << endl;
    return result;
  }
  if (globalData->debug) cout << b << " ";
  result.dataLength=b;

  // data
  for (int i=0;i<result.dataLength;i++) {
    b = getByte();
    if (b == -1) {
      // datas that are not received will be skipped
      result.dataLength--;
      i--;
      continue;
    } else {
      result.data[i]=b;
      if (globalData->debug) cout << b << " ";
    }
  }
  // insert receive of crc byte here
  result.commSuccess=true;
  if (globalData->debug) cout << endl;
  return result;
}

void ECBCommunicator::initialise() {
  //  globalData->comm=this;
  if (globalData->debug)
    std::cout << "ECBCommunicator: (external) initialising..." << std::endl;
}





/*
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

  globalData->configs.push_back(communication);
  globalData->configs.push_back(controller);
  globalData->agents.push_back(communication->getAgent());

  showParams(globalData->configs);

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
}*/
