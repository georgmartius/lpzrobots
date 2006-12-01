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
 *   Revision 1.5  2006-12-01 16:18:42  martius
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


using namespace lpzrobots;


class ThisSim : public Simulation {
public:
	

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    setCameraHomePos(Pos(-10.7854, -7.41751, 5.92078),  Pos(-50.6311, -5.00218, 0));

    //****************/
    SchlangeConf conf = Schlange::getDefaultConf();
    conf.motorPower=0.2;
    conf.frictionJoint=0.01;
    conf.segmNumber=6; 
    //     conf.jointLimit=conf.jointLimit*3;
    SchlangeServo2* schlange1 = 
      new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
			  conf, "S1");
    ((OdeRobot*)schlange1)->place(Pos(0,0,3)); 

    //AbstractController *controller = new InvertNChannelController(100/*,true*/);  
    //  AbstractController *controller = new InvertMotorSpace(100/*,true*/);  
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.cInit=0.5;
    cc.useS=true;
    AbstractController *controller = new InvertMotorNStep(cc);  
    //    AbstractController *controller = new SineController();  
  
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
  
 
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("gravity", 0.0); 

    controller->setParam("steps",2);
    controller->setParam("epsC",0.0005);
    controller->setParam("epsA",0.01);
    controller->setParam("adaptrate",0);
    //    controller->setParam("rootE",3);

    // controller->setParam("desens",0.0);
    //   controller->setParam("s4delay",1.0);
    //   controller->setParam("s4avg",1.0);
    
    //   controller->setParam("factorB",0.0);
    //   controller->setParam("zetaupdate",0.1);

    Primitive* head = schlange1->getMainPrimitive();
    Joint* j = new BallJoint(head, global.environment, head->getPosition());
    j->init(odeHandle, osgHandle);

    showParams(global.configs);
  }

};


int main (int argc, char **argv)
{  
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}


 
  
