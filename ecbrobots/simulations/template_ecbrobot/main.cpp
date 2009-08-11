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
 *   Revision 1.6  2009-08-11 15:50:19  guettler
 *   Current development state:
 *   - Support of communication protocols for XBee Series 1, XBee Series 2 and cable mode
 *   - merged code base from ecb_robots and Wolgang Rabes communication handling;
 *     ECBCommunicator (almost) entirely rewritten: Use of Mediator (MediatorCollegues: ECB),
 *     Callbackble (BackCaller: SerialPortThread)
 *   - New CThread for easy dealing with threads (is using pthreads)
 *   - New TimerThreads for timed event handling
 *   - SerialPortThread now replaces the cserialthread
 *   - GlobalData, ECBCommunicator is now configurable
 *   - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *   - ECBRobot is now Inspectables (uses new infoLines functionality)
 *   - ECB now supports dnsNames and new communication protocol via Mediator
 *   - Better describing command definitions
 *   - SphericalRobotECB: adapted to new ECB structure, to be tested
 *
 *   Revision 1.5  2008/07/16 14:37:17  robot1
 *   -simple getc included
 *   -extended config on reset
 *   -minor changes
 *
 *   Revision 1.4  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.3  2008/104/11 06:31:16  guettler
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

#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <selforg/abstractcontrolleradapter.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

using namespace std;

#include "ecb.h"
#include "ecbcommunicator.h"
#include "commanddefs.h"
/**
 * Simple cheat to get motors motorsStopped without implemented start and stop command.
 */
class CheatedECB : public ECB {
  public:
    CheatedECB(string dnsName, GlobalData& globalData, ECBConfig& ecbConfig) :
      ECB(dnsName, globalData, ecbConfig), motorsStopped(false) {
    }

    virtual ~CheatedECB() {
    }

    virtual void sendMotorValuesPackage() {
      // at first start of ECB, it must be initialized with a reset-command
      assert(initialised && failureCounter <= globalData->maxFailures);

      if (globalData->debug)
        cout << "ECB(" << dnsName << "): sendMotorPackage()!" << endl;

      // prepare the communication-protocol
      ECBCommunicationEvent* event = new ECBCommunicationEvent(ECBCommunicationEvent::EVENT_REQUEST_SEND_COMMAND_PACKAGE);

      event->commPackage.command = COMMAND_MOTORS;
      event->commPackage.dataLength = currentNumberMotors;
      // set motor-data
      int i = 0;
      // motorList was update by ECBAgent->ECBRobot:setMotors()->(to all ECBs)-> ECB:setMotors()
      FOREACH (list<motor>,motorList,m) {
        // Agent and Controller process with double-values
        // The ECB(hardware) has to work with byte-values
        if (motorsStopped)
          event->commPackage.data[i++] = convertToByte(0);
        else
          event->commPackage.data[i++] = convertToByte((*m));
      }
      informMediator(event);
    }

    /**
     * Send stop command to the ECB to disable the motors
     */
    virtual void stopMotors() {
      motorsStopped=true;
      if (initialised && failureCounter <= globalData->maxFailures)
        sendMotorValuesPackage();
    }

    /**
     * Send start command to the ECB to enable the motors
     */
    virtual void startMotors() {
      motorsStopped=false;
    }

  private:
    bool motorsStopped;
};

class MyController : public AbstractControllerAdapter {
  public:
    MyController(AbstractController* controller) : AbstractControllerAdapter(controller) {

    }

    virtual void step(const sensor* sensors, int sensornumber,
          motor* motors, int motornumber) {
      controller->step(sensors, sensornumber, motors,  motornumber);
     // motors[0] = 0;
      //motors[1] = -1;
    }
};

class MyECBManager : public ECBManager {

  public:

    MyECBManager() {
    }

    virtual ~MyECBManager() {
    }

    /**
     * This function is for the initialisation of
     * ECBagents, ECBRobots and their heart, the Controller.
     * @param global
     * @return true if initialisation was successful
     */
    virtual bool start(GlobalData& global) {

      // set specific communication values
      global.baudrate = 57600;
      global.portName = "/dev/ttyUSB0";
      global.maxFailures = 4;
      global.serialReadTimeout = 100;
      global.verbose = true;
      global.debug = true;
      global.cycleTime = 50;
      global.noise = 0.05;
      global.plotOptions.push_back(PlotOption(GuiLogger, 1));

      // create new controller
     // BasicControllerConf conConf = BasicController::getDefaultConf();
     // AbstractController* myCon1 = new MyController(new BasicController(conConf));
      AbstractController* myCon1 = new InvertMotorNStep(InvertMotorNStep::getDefaultConf());
   //    AbstractController* myCon1 = new SineController();
      myCon1->setParam("period",150);
      // create new wiring
      AbstractWiring* myWiring1 = new One2OneWiring(new WhiteNormalNoise());
      // create new robot
      ECBRobot* myRobot1 = new ECBRobot(global);
      // 2 ECBs will be added to robot
      ECBConfig ecbc1 = ECB::getDefaultConf();
//      ECB* myECB1 = new CheatedECB("NIMM2_PRIMUS", global, ecbc1);
      ECB* myECB1 = new CheatedECB(" NIMM2_SECUNDUS", global, ecbc1);
//      ECB* myECB1 = new CheatedECB("NIMM2_TERTIUS", global, ecbc1);
//      ECB* myECB1 = new CheatedECB("XBEE_S1_ECB", global, ecbc1);
//      ECB* myECB1 = new CheatedECB("XBEE_S1_MODUL_00", global, ecbc1);
      myRobot1->addECB(myECB1);
      // ECBConfig ecbc2 = ECB::getDefaultConf();
      //myRobot1->addECB ( 2,ecbc2 );

      // create new agent
      ECBAgent* myAgent1 = new ECBAgent(PlotOption(GuiLogger_File, 5), global.noise);
      // init agent with controller, robot and wiring
      myAgent1->init(myCon1, myRobot1, myWiring1);

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
      global.agents.push_back(myAgent1);
      //  global.agents.push_back ( myAgent2 );

      return true;
    }

    /** optional additional callback function which is called every simulation step.
     Called between physical simulation step and drawing.
     @param pause indicates that simulation is paused
     @param control indicates that robots have been controlled this timestep
     */
    virtual void addCallback(GlobalData& globalData, bool pause, bool control) {

    }

    /** add own key handling stuff here, just insert some case values
     *
     * @param globalData
     * @param key
     * @return
     */
    virtual bool command(GlobalData& globalData, int key) {
      return false;
    }

};

/**
 * normally here do not change anything
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
  MyECBManager ecb;
  return ecb.run(argc, argv) ? 0 : 1;
}

