/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.2  2006-07-14 12:23:55  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.5  2006/06/26 20:36:17  robot5
 *   Initial value of variables modified.
 *
 *   Revision 1.1.2.4  2006/06/20 07:17:35  robot3
 *   -changed some behaviour of wheelie
 *   -added cvs log
 *
 *
 ***************************************************************************/
#include "simulation.h"

#include "odeagent.h"
#include "playground.h"
#include "passivesphere.h"

#include <selforg/invertmotornstep.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include "wheelie.h"
#include "sliderwheelie.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("gravity",-1); // normally at -9.81
    // initialization
    
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(25, 0.2, 1.5));
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);
    
    for(int i=0; i<5; i++){
      PassiveSphere* s = 
	new PassiveSphere(odeHandle, 
			  osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
      s->setPosition(Pos(i*0.5-2, i*0.5, 1.0)); 
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);    
    }
        
    OdeAgent *agent, *slideragent;
    AbstractWiring *wiring, *sliderwiring;
    AbstractController *controller, *slidercontroller;

    
    Wheelie *myWheelie;
    WheelieConf myWheelieConf = DefaultWheelie::getDefaultConf();
    /******* w H E E L I E *********/
    myWheelieConf.jointLimit=M_PI/4;
    myWheelieConf.motorPower=0.2;
    myWheelieConf.frictionGround=0.04;
    myWheelie = new Wheelie(odeHandle, osgHandle, myWheelieConf, "Wheelie1");
    ((OdeRobot*) myWheelie)->place(Pos(-5,-5,0.2)); 
    InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
    controller = new InvertMotorNStep(invertnconf);    
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    agent = new OdeAgent(plotoptions);
    agent->init(controller, myWheelie, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);
    global.configs.push_back(myWheelie);   
    myWheelie->setParam("gamma",/*gb");
    global.obstacles.push_back(s)0.0000*/ 0.0);


    SliderWheelie *mySliderWheelie;
    SliderWheelieConf mySliderWheelieConf = DefaultSliderWheelie::getDefaultConf();
    /******* S L I D E R - w H E E L I E *********/
    mySliderWheelieConf.jointLimit=M_PI/2;
    mySliderWheelieConf.motorPower=0.2;
    mySliderWheelieConf.frictionGround=0.04;
    mySliderWheelie = new SliderWheelie(odeHandle, osgHandle, mySliderWheelieConf, "sliderWheelie1");
    ((OdeRobot*) mySliderWheelie)->place(Pos(-5,-4,0.2)); 
    InvertMotorNStepConf sliderinvertnconf = InvertMotorNStep::getDefaultConf();
    slidercontroller = new InvertMotorNStep(sliderinvertnconf);    
    sliderwiring = new One2OneWiring(new ColorUniformNoise(0.1));
    slideragent = new OdeAgent(plotoptions);
    slideragent->init(slidercontroller, mySliderWheelie, sliderwiring);
    global.agents.push_back(slideragent);
    global.configs.push_back(slidercontroller);
    global.configs.push_back(mySliderWheelie);   
    mySliderWheelie->setParam("gamma",/*gb");
    global.obstacles.push_back(s)0.0000*/ 0.0);


  }
  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	default:
	  return false;
	  break;
	}
    }
    return false;
  }
  
};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
