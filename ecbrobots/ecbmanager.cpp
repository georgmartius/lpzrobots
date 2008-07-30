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
 *   Revision 1.7  2008-07-30 06:03:47  robot1
 *   the new directory GUIs is added
 *
 *   Revision 1.6  2008/07/16 15:16:55  robot1
 *   minor bugfixes
 *
 *   Revision 1.5  2008/07/16 14:37:17  robot1
 *   -simple getc included
 *   -extended config on reset
 *   -minor changes
 *
 *   Revision 1.4  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.3  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.2  2008/04/08 10:11:03  guettler
 *   alpha testing
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/

#include "ecbmanager.h"

#include <selforg/agent.h>

#include "ecbcommunicator.h"
#include "globaldata.h"
#include "console.h"
#include "cmdline.h"

#include "curses.h"

namespace lpzrobots {


ECBManager::ECBManager() : Configurable("ECBManager","$ID$"),simulation_time_reached(false), commInitialized(false) {

  globalData.configs.push_back ( this );

}

ECBManager::ECBManager(ECBCommunicator* comm) : Configurable("ECBManager","$ID$"),simulation_time_reached(false), comm(comm), commInitialized(true) {
  globalData.configs.push_back ( this );
  
}


ECBManager::~ECBManager() { // TODO: aufräumen
}

void ECBManager::handleStartParameters ( int argc, char** argv ) {
  if ( contains ( argv,argc,"-g" ) !=0 ) globalData.plotOptions.push_back ( PlotOption ( GuiLogger ) );

  if ( contains ( argv,argc,"-f" ) !=0 ) globalData.plotOptions.push_back ( PlotOption ( File ) );

  if ( contains ( argv,argc,"-s" ) !=0 ) globalData.plotOptions.push_back ( PlotOption ( SoundMan ) );

  if ( contains ( argv,argc,"-d" ) !=0 ) {
    globalData.debug=true;
    globalData.verbose=true;
  }

  if ( contains ( argv,argc,"-v" ) !=0 ) globalData.verbose=true;

  if ( contains ( argv,argc,"-bench" ) !=0 ) globalData.benchmarkMode=true;

  if ( contains ( argv,argc,"-h" ) !=0 ) {
    printf ( "Usage: %s [-g] [-f] [-s] [-b] [-p port] [-v] [-d] [-bench] [-h]\n",argv[0] );
    printf ( "\t-g\tstart guilogger\n" );
    printf ( "\t-f\twrite logfile\n" );
    printf ( "\t-s\tenable SoundMan\n" );
    printf ( "\t-b baud\tset baud rate\n" );
    printf ( "\t-p port\tuse given serial port ( e.g. /dev/ttyUSB0 ) \n" );
    printf ( "\t-v\tenable verbose mode\n" );
    printf ( "\t-d\tenable debug mode (displays more information than verbose)\n" );
    printf ( "\t-bench\tenable benchmark mode\n" );
    printf ( "\n\t-h\tdisplay this help\n" );
    exit ( 0 );
  }

  int index = contains ( argv,argc,"-p" );

  if ( index && index<argc ) {
    globalData.portName = std::string(argv[index]);
    cout << "Using port " << globalData.portName << endl;
  }

  index = contains ( argv,argc,"-b" );

  if ( index && index<argc ) {
    globalData.baudrate = atoi ( argv[index] );
    cout << "Using baud rate " << globalData.baudrate << endl;
  }
}

bool ECBManager::run ( int argc, char** argv ) {

  // params that may affect the start function
  if ( contains ( argv,argc,"-d" ) !=0 ) globalData.debug=true;
  if ( contains ( argv,argc,"-v" ) !=0 ) globalData.verbose=true;

  // first call the start function
  if (globalData.debug) cout << "ECBManager: calling start function..." << endl;
  start ( globalData );

  if (globalData.debug) cout << "ECBManager: handling console parameters..." << endl;
  handleStartParameters ( argc,argv );

  // initialise Port for RS232, start the Communicator
  // neccessary values are in globalData stored
  if (globalData.debug) cout << "ECBManager: initialising ECBCommunicator..." << endl;
  
if (!commInitialized) {
    comm = new ECBCommunicator ( globalData );
    commInitialized=true;
  } else
    comm->setConfig(globalData);

  // init console...defined in console.h
  if (globalData.debug) cout << "ECBManager: initialising console..." << endl;
  initializeConsole();

  if (globalData.debug) cout << "ECBManager: starting communication..." << endl;
  globalData.comm->start();

  // run the loop
  if (globalData.debug) cout << "ECBManager: starting the loop..." << endl;
  while ( (!simulation_time_reached) && globalData.comm->is_running()) {
    //cout << "sim_time_reached: " << simulation_time_reached << endl;


    //disable line buffering
    cbreak();


	if ( !loop() )
     	  break;
  }
  if (globalData.debug) cout << "ECBManager: loop finished." << endl;

  // close all things which where used
  if (globalData.debug) cout << "ECBManager: closing console..." << endl;
  closeConsole();

  if (globalData.debug) cout << "ECBManager: Reaching the final end. Bye!" << endl;
  return true;
}

bool ECBManager::loop() {
//     if (globalData.debug) cout << "ECBManager: running one loop step..." << endl;
    // check for cmdline interrupt
    if ( control_c_pressed() ) {
      globalData.pause=true;
      usleep ( 200000 );

      if ( !handleConsole ( globalData ) ) {
        globalData.comm->stopandwait();
      }

      cmd_end_input();

      globalData.pause=false;
    }
  
  char taste = getch();
    if ( taste > 10 ) {
      std::cout << ".";
      command(this->globalData,taste);
    }
  return true;
}


/// CONFIGURABLE INTERFACE

Configurable::paramval ECBManager::getParam(const paramkey& key) const{
  /*if(key == "noise") return noise;
  else if(key == "cycletime") return cycletime;
  else if(key == "reset") return 0;
  else */ return Configurable::getParam(key);
}

bool ECBManager::setParam(const paramkey& key, paramval val){
  /*if(key == "noise") noise = val;
  else if(key == "cycletime"){
    cycletime=(long)val;
  } else if(key == "reset"){
    doReset=true;
  } else*/
    return Configurable::setParam(key, val);
  //return true;
}

Configurable::paramlist ECBManager::getParamList() const {
  paramlist list;
/*  list += pair<paramkey, paramval> (string("noise"), noise);
  list += pair<paramkey, paramval> (string("cycletime"), cycletime);
  list += pair<paramkey, paramval> (string("reset"), 0);*/
  return list;
}

}

