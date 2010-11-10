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
 *   Revision 1.8  2010-11-10 09:31:24  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.7  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.6  2008/08/12 12:22:50  guettler
 *   added new custom MySimpleController for controlling motor values due to keyboard
 *
 *   Revision 1.5  2008/08/12 11:48:30  guettler
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
#include <selforg/sinecontroller.h>
#include <selforg/abstractcontrolleradapter.h>
#include <selforg/configurable.h>

#include "SphericalRobotECB.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;



class MySimpleController : public AbstractControllerAdapter
{
  public:

    MySimpleController ( AbstractController* controller, GlobalData& global ) :AbstractControllerAdapter ( controller ) {
      global.configs.push_back ( this );
      global.configs.push_back ( controller );

      this->addParameterDef ( "speedFactor", &speedFactor, 0.5 );
    }

    /// return the name of the object
    /// @override
    virtual paramkey getName() const {
      return "MySimpleController";
    }

    /** initialisation of the controller with the given sensor/ motornumber
     Must be called before use. The random generator is optional.
    */
    virtual void init ( int sensornumber, int motornumber, RandGen* randGen = 0 ) {
      AbstractControllerAdapter::init ( sensornumber, motornumber );
      
      controller_enabled = true;
    }

    /** performs one step (includes learning).
        Calculates motor commands from sensor inputs.
        @param sensors sensors inputs scaled to [-1,1]
        @param sensornumber length of the sensor array
        @param motors motors outputs. MUST have enough space for motor values!
        @param motornumber length of the provided motor array
    */
    virtual void step ( const sensor* sensors, int sensornumber, motor* motors, int motornumber ) {
      if ( controller_enabled ) {
        AbstractControllerAdapter::step ( sensors,sensornumber, motors, motornumber );
        for ( int i=0; i<motornumber; i++ ) {
          motors[i]*=speedFactor;
        }
      } else {
        stepNoLearning ( sensors,sensornumber,motors,motornumber );
      }
    }

    /** performs one step without learning.
        @see step
    */
    virtual void stepNoLearning ( const sensor* sensors, int number_sensors, motor* motors, int number_motors ) {
      motors[0] = convertToDouble(motorValues[0]);
      motors[1] = convertToDouble(motorValues[1]);
    }

    double convertToDouble ( int byteVal ) {
      return ( ( ( double ) ( byteVal-127 ) ) /128. ); //values -1..0..+1
    }

    int convertToByte ( double doubleVal ) {
      // insert check byteVal<255
      int byteVal = ( int ) ( ( doubleVal+1.) *128.0 );
      return ( byteVal<255?byteVal:254 ); //values 0..128..256
    }

//   sensor* sensorValues;
  int motorValues[4];
  bool controller_enabled;
  
  private:
    /* Define the Configurable params */
    Configurable::paramval speedFactor;

    /* Define the Inspectable params */
    //...


};


class MyECBManager : public ECBManager
{

  public:

    MySimpleController* myCon;


    MyECBManager() {}

    virtual ~MyECBManager() {}

    /**
     * This function is for the initialising of agents and robots,...
     * @param global
     * @return true if initialisation was successful
     */
    virtual bool start ( GlobalData& global ) {

      /// global values ***************************************
      global.baudrate = 38400;
//       global.portName = "/dev/ttyS0";
      global.portName = "/dev/ttyUSB0";
      global.masterAddress=0;
      global.maxFailures=10;
      global.serialReadTimeout=50;
      global.verbose = false;
      global.debug = true;

      motors_stopped = false;
      
      /// create new controller *******************************
      AbstractController* acon = new SineController ( 2 );
      acon->setParam ( "sinerate", 15 );
      acon->setParam ( "phaseshift", 45 );
      myCon = new MySimpleController ( acon,global );
      
      myCon->motorValues[0]=127;
      myCon->motorValues[1]=127;
      
      myCon->setParam("speedFactor",1.0);
      
      /// create new wiring ***********************************
      AbstractWiring* myWiring1 = new One2OneWiring ( new WhiteNormalNoise() );
      
      /// create new robot ************************************
      myRobot1 = new ECBRobot ( global );
      
      /// add ECB_1 that controlls motors and get values of two 
      /// tilt-sensors and two infrared-sensors to the SphericalRobot
      ECBConfig ecbc1 = ECB::getDefaultConf();
      //SphericalRobotECB* sp_robot = new SphericalRobotECB (/*addr*/ 1, global, ecbc1 );
      //myRobot1->addECB ( sp_robot );
      
      myRobot1->addECB ( 1,ecbc1 );
    
      /// add ECB_2 that collect the infrared-sensors of 
      /// the outer shell of the SphericalRobot
//       ECBConfig ecbc2 = ECB::getDefaultConf();
//       myRobot1->addECB ( 2,ecbc2 );

      /// create and init a new agent *************************
      ECBAgent* myAgent1 = new ECBAgent ( PlotOption ( ECBRobotGUI ) );
      myAgent1->init ( myCon,myRobot1,myWiring1 );
      /// add the GuiLogger and ECBRobotGUI for visualisation
      PlotOption po(GuiLogger);
      myAgent1->addPlotOption(po);
      PlotOption po2(File);
      myAgent1->addPlotOption(po2);

      // register agents
      global.agents.push_back ( myAgent1 );

      return true;
    }

    /** optional additional callback function which is called every simulation step.
    Called between physical simulation step and drawing.
    @param paused indicates that simulation is paused
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
    virtual bool command ( GlobalData& globalData, int key ) {

//       printf("\r\nmotors: %d %d\r\n",myCon->motorValues[0],myCon->motorValues[1]);

      switch ( key ) {
        case '6': //forward
          if ( myCon->motorValues[0]<=255 ) {
            myCon->motorValues[0] += 8;
          }
          if ( myCon->motorValues[1]<256 ) {
            myCon->motorValues[1] += 8;
          }
          break;
        case '4': //backward
          if ( myCon->motorValues[0]>=0 ) {
            myCon->motorValues[0] -= 8;
          }
          if ( myCon->motorValues[1]>=0 ) {
            myCon->motorValues[1] -= 8;
          }
          break;
        case '8': //left
          if ( myCon->motorValues[1]<=255 ) {
            myCon->motorValues[1] += 8;
          }
          if ( myCon->motorValues[0]<256 ) {
            myCon->motorValues[0] -= 8;
          }
          break;
        case '2': //right
          if ( myCon->motorValues[1]>=0 ) {
            myCon->motorValues[1] -= 8;
          }
          if ( myCon->motorValues[0]>=0 ) {
            myCon->motorValues[0] += 8;
          }
          break;
        case '5'://stop
          myCon->motorValues[0]=127;
          myCon->motorValues[1]=127;
          break;
      case ' ':// toggle between motors-on/-off
          if ( motors_stopped ) myRobot1->stopMotors();
          else myRobot1->startMotors();
          motors_stopped=!motors_stopped;
          break;
      case '+': // activate external selforg-controller (KI)
          myCon->controller_enabled=true;
          break;
      case '-': // disable selforg-controller to controll robot manual by key-pressing
          myCon->controller_enabled=false;
          break;

      case 'r':  // make a reset of all
          myRobot1->resetECBs();
          break;
      default:
//           return true;
          break;
      }
      return false;
    }

  private:
    ECBRobot* myRobot1;
    bool motors_stopped;

};


/**
 * normally here do not change anything
 * @param argc
 * @param argv
 * @return
 */
int main ( int argc, char **argv )
{
  MyECBManager ecb;
  return ecb.run ( argc, argv ) ? 0 : 1;
}



