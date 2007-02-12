/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.7  2007-02-12 13:25:47  martius
 *   on the way to teaching
 *
 *   Revision 1.6  2007/01/26 12:07:08  martius
 *   orientationsensor added
 *
 *   Revision 1.5  2006/12/01 16:18:42  martius
 *   test of S
 *
 *   Revision 1.4  2006/08/04 16:25:14  martius
 *   bugfixing
 *
 *   Revision 1.3  2006/07/14 12:23:52  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.2.4.5  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.2.4.4  2006/03/29 15:10:22  martius
 *   *** empty log message ***
 *
 *   Revision 1.2.4.3  2006/02/24 14:43:51  martius
 *   keys
 *
 *   Revision 1.2.4.2  2005/12/29 16:44:54  martius
 *   moved to osg
 *
 *   Revision 1.2.4.1  2005/11/15 12:29:59  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.2  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/


#include <selforg/noisegenerator.h>

#include "simulation.h"
#include "odeagent.h"
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include "playground.h"
#include "passivesphere.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>

#include "schlangeservo2.h"
#include "schlangeservo.h"


using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  Joint* fixator;
  InvertMotorNStep* controller;
  bool teaching;
  bool dteaching;
  double* teachingSignal;
  int teachingLen;
  double* dteachingSignal;
  int dteachingLen;
  

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    setCameraHomePos(Pos(-19.7951, -12.3665, 16.4319),  Pos(-51.7826, -26.772, 0));

    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("gravity", 0.0); 
    global.odeConfig.setParam("realtimefactor",4);

    //****************/
    SchlangeConf conf = Schlange::getDefaultConf();
    conf.motorPower=0.5;
    conf.frictionJoint=0.001;
    conf.segmNumber=15;     
    //     conf.jointLimit=conf.jointLimit*3;
    SchlangeServo* schlange1 = 
      new SchlangeServo ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
			  conf, "Schlange1D");
    ((OdeRobot*)schlange1)->place(Pos(0,0,3)); 

    //    //AbstractController *controller = new InvertNChannelController(100/*,true*/);  
    //  AbstractController *controller = new InvertMotorSpace(100/*,true*/);  
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.cInit=1.0;
    cc.cNonDiag=0.0;
    //    cc.useS=true;
    //    cc.someInternalParams=false;
    controller = new InvertMotorNStep(cc);  
    //AbstractController *controller = new SineController();  
    
  
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    //   DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
    //   c.useId=true;
    //   c.useFirstD=true;
    //   // c.useSecondD=true;
    //   c.derivativeScale=10;
    //   AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));
    OdeAgent* agent = new OdeAgent(plotoptions);
    agent->init(controller, schlange1, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);
    global.configs.push_back(schlange1);
  

    //controller->setParam("inhibition",0.00);
    controller->setParam("limitrf",3);
    //    controller->setParam("kwta",5);
    controller->setParam("dampS",0.001);
 
    controller->setParam("rootE",3);
    controller->setParam("steps",1);
    controller->setParam("epsC",0.00);
    controller->setParam("epsA",0.00);
    controller->setParam("adaptrate",0.000);
    //    controller->setParam("nomupdate",0.05);

    // controller->setParam("desens",0.0);
    //   controller->setParam("s4delay",1.0);
    controller->setParam("s4avg",2.0);
    
    //   controller->setParam("factorB",0.0);
    //   controller->setParam("zetaupdate",0.1);

    Primitive* head = schlange1->getMainPrimitive();
    fixator = new BallJoint(head, global.environment, head->getPosition());
    fixator->init(odeHandle, osgHandle);

    showParams(global.configs);
    
    teaching=false;
    dteaching=false;
    teachingLen = schlange1->getMotorNumber();
    teachingSignal = new double[teachingLen];
    dteachingLen = schlange1->getSensorNumber();
    dteachingSignal = new double[teachingLen];
  }


  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    double sineRate=30;
    double phaseShift=0.65;
    if(teaching){
      for(int i=0; i<teachingLen; i++){
	teachingSignal[i]=sin(globalData.time/sineRate + i*phaseShift*M_PI/2);
      }

    }

  };

  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;    
    bool handled = false;
    FILE* f;
    //    double m;
    //    motor motors[2];
    switch ( key )
      {
      case 't' : 
	teaching=!teaching;
	if(teaching) dteaching=false;
	printf("Teaching Signal: %s,\n", teaching ? "on" : "off");
	handled = true; 
	break;
      case 'j' : 
	dteaching=!dteaching;
	if(dteaching) teaching=false;
	printf("Distal Teaching Signal: %s,\n", dteaching ? "on" : "off");
	handled = true; 
	break;
      case 's' :
        f = fopen("test","wb");
	controller->store(f) && printf("Controller stored\n");
	fclose(f);
	handled = true; break;	
      case 'l' :
	f = fopen("test","rb");
	controller->restore(f) && printf("Controller loaded\n");
	fclose(f);
	handled = true; break;	
      }
    fflush(stdout);
    return handled;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Teaching: t","toggle motor teaching");
    au.addKeyboardMouseBinding("Teaching: d","toggle distal teaching");
    au.addKeyboardMouseBinding("Simulation: s","store");
    au.addKeyboardMouseBinding("Simulation: l","load");
  }

};




int main (int argc, char **argv)
{  
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}


 
  
