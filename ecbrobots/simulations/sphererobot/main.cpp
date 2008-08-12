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
 *   Revision 1.6  2008-08-12 12:22:50  guettler
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

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class MySimpleController : public AbstractController {
	public:

		  MySimpleController();

		  /** initialisation of the controller with the given sensor/ motornumber
			  Must be called before use. The random generator is optional.
		  */
		  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0) {
			  this->sensorNumber=sensornumber;
			  this->motorNumber=motornumber;
			  this->motorvalues = (motor*) malloc(sizeof(motor)*motornumber);
			  this->sensorvalues = (sensor*) malloc(sizeof(motor)*motornumber);
			  this->addParameter("sinerate",&sineRate);
			  this->addParameter("phaseShift",&phaseShift);
			  sineRate=40;
			  phaseShift=1;
		  }

		  /** @return Number of sensors the controller was initialised
		      with or 0 if not initialised */
		  virtual int getSensorNumber() const {return sensorNumber;}


		  /** @return Number of motors the controller was initialised
		      with or 0 if not initialised */
		  virtual int getMotorNumber() const {return motorNumber;}

		  /** performs one step (includes learning).
		      Calculates motor commands from sensor inputs.
		      @param sensors sensors inputs scaled to [-1,1]
		      @param sensornumber length of the sensor array
		      @param motors motors outputs. MUST have enough space for motor values!
		      @param motornumber length of the provided motor array
		  */
		  virtual void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
			  stepNoLearning(sensors,sensornumber,motors,motornumber);
		  }

		  /** performs one step without learning.
		      @see step
		  */
		  virtual void stepNoLearning(const sensor* , int number_sensors, motor* , int number_motors) {
			  // write internal values into motor array
			  for (int i=0; i<number_motors; i++){
				  motors[0]=motorValue[0];
				  //motors[i]=sin(t/sineRate + i*phaseShift*M_PI/2); // sinecontroller
			  }
		  }

		  virtual paramval getParam(const paramkey& key) const {

		  }

		  virtual bool setParam(const paramkey& key, paramval val) {

		  }

		  virtual paramlist getParamList() const  {

		  }

		  virtual bool store(FILE* f) const { return true;  }

		  virtual bool restore(FILE* f) const { return true;  }


     int t;
	 std::string name;
	 motor* motorValues;
	 sensor* sensorValues;
	 int sensorNumber;
	 int motorNumber;
	 paramval* sineRate;
	 paramval* phaseShift;
};


class MyECBManager : public ECBManager {

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

      // set specific communication values
      global.baudrate = 115200;
      global.portName = "/dev/ttyS0";
      global.masterAddress=0;
      global.maxFailures=4;
      global.serialReadTimeout=50;
      global.verbose = true;
      global.debug = true;


      // create new controller
      myCon = new MySimpleController();
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
      myAgent1->init ( myCon,myRobot1,myWiring1 );

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
      case 127:
    	  myCon->motorValues[0]++;
    	  break;
      case 115:
    	  myCon->motorValues[0]++;
    	  break;
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



