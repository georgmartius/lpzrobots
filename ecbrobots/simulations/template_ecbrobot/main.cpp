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
 *   Revision 1.3  2008-04-11 06:31:16  guettler
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

#include <selforg/invertmotorspace.h>
#include <selforg/one2onewiring.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class MyECBManager : public ECBManager {

  public:

    MyECBManager() {}

    virtual ~MyECBManager() {}

    /**
     * This function is for the initialising of agents and robots,...
     * @param global
     * @return true if initialisation was successful
     */
    virtual bool start ( GlobalData& global ) {

      // set specific communication values
      global.baudrate = 38400;
      global.portName = "/dev/ttyS0";
      global.masterAddress=0;
      global.maxFailures=4;
      global.serialReadTimeout=5;
      global.verbose = true;


      // create new controller
      AbstractController* myCon1 = new InvertMotorSpace ( 10 );
      // create new wiring
      AbstractWiring* myWiring1 = new One2OneWiring ( new WhiteNormalNoise() );
      // create new robot
      ECBRobot* myRobot1 = new ECBRobot ( global );
      // 2 ECBs will be added to robot
      ECBConfig ecbc1 = ECB::getDefaultConf();
      myRobot1->addECB ( 1,ecbc1 );
      // ECBConfig ecbc2 = ECB::getDefaultConf();
      //myRobot1->addECB ( 2,ecbc2 );

      // create new agent
      ECBAgent* myAgent1 = new ECBAgent();
      // init agent with controller, robot and wiring
      myAgent1->init ( myCon1,myRobot1,myWiring1 );

      // create new controller with example parameter changes
//       AbstractController* myCon2 = new InvertMotorSpace ( 10 );
//       myCon2->setParam ( "s4delay",2.0 );
//       myCon2->setParam ( "s4avg",2.0 );
//       // create new wiring
//       AbstractWiring* myWiring2 = new One2OneWiring ( new WhiteNormalNoise() );
//       // create new robot
//       ECBRobot* myRobot2 = new ECBRobot ( global );
//       // 2 ECBs will be added to robot
//       ECBConfig ecbc3 = ECB::getDefaultConf();
//       myRobot1->addECB ( 3,ecbc3 );
//       ECBConfig ecbc4 = ECB::getDefaultConf();
//       myRobot1->addECB ( 4,ecbc4 );
//
//       // create new agent
//       ECBAgent* myAgent2 = new ECBAgent();
//       // init agent with controller, robot and wiring
//       myAgent2->init ( myCon2,myRobot2,myWiring2 );



      // register agents
      global.agents.push_back ( myAgent1 );
      //  global.agents.push_back ( myAgent2 );

      return true;
    }

    /** optional additional callback function which is called every simulation step.
    Called between physical simulation step and drawing.
    @param pause indicates that simulation is paused
    @param control indicates that robots have been controlled this timestep
    */
    virtual void addCallback ( GlobalData& globalData,bool pause, bool control ) {

    }

    /** add own key handling stuff here, just insert some case values
     *
     * @param globalData
     * @param key
     * @param down
     * @return
     */
    virtual bool command ( GlobalData& globalData, int key, bool down ) {
      return false;
    }


};


/**
 * normally here do not change anything
 * @param argc
 * @param argv
 * @return
 */
int main ( int argc, char **argv ) {
  MyECBManager ecb;
  return ecb.run ( argc, argv ) ? 0 : 1;
}



