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
 *   Revision 1.5  2008-08-12 11:48:30  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.3  2008/07/16 14:37:17  robot1
 *   -simple getc included
 *   -extended config on reset
 *   -minor changes
 *
 *   Revision 1.2  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.1  2008/04/11 10:11:08  guettler
 *   Added ECBManager for the SphericalRobot
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
      global.baudrate = 115200;
      global.portName = "/dev/ttyS0";
      global.masterAddress=0;
      global.maxFailures=4;
      global.serialReadTimeout=50;
      global.verbose = true;
      global.debug = true;


      // create new controller
      AbstractController* myCon1 = new InvertMotorSpace ( 10 );
      // create new wiring
      AbstractWiring* myWiring1 = new One2OneWiring ( new WhiteNormalNoise() );
      // create new robot
      ECBRobot* myRobot1 = new ECBRobot ( global );
      // 2 ECBs will be added to robot
      ECBConfig ecbc1 = ECB::getDefaultConf();
      myRobot1->addECB ( 1,ecbc1 );
      ECBConfig ecbc2 = ECB::getDefaultConf();
      ecb2.adcTypes[0] = ADC_TILT;
      ecb2.adcTypes[1] = ADC_TILT;
      ecb2.adcTypes[2] = ADC_IR;
      ecb2.adcTypes[3] = ADC_IR;
      myRobot1->addECB ( 2,ecbc2 );

      // create new agent
      ECBAgent* myAgent1 = new ECBAgent(PlotOption(ECBRobotGUI));
      // init agent with controller, robot and wiring
      myAgent1->init ( myCon1,myRobot1,myWiring1 );

      // register agents
      global.agents.push_back ( myAgent1 );

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
     * @return
     */
    virtual bool command ( GlobalData& globalData, int key) {
      switch (key) {
        //        case 117:
        //        return true;
        //          break;
        default:
          std::cout << "key pressed: " << key << std::endl;
        return true;
          break;
      }
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



