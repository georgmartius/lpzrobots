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
 *   Revision 1.2  2006-08-11 15:46:12  martius
 *   two spheres
 *
 *   Revision 1.1  2006/08/08 17:04:47  martius
 *   added new sensor model
 *
 *
 ***************************************************************************/

// include simulation environment stuff
#include "simulation.h"

// include agent (class for holding a robot, a controller and a wiring)
#include "odeagent.h"
#include "playground.h" // arena
#include "passivebox.h"  // passive box

// controller
//#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>


#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring
#include <selforg/derivativewiring.h>

// robots
#include "forcedsphere.h"
#include "axisorientationsensor.h"
#include "relativepositionsensor.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  AbstractController* controller;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(19.8417, -11.473, 11.4588),  Pos(53.69, -25.548, 0));
    // initialization
    // - set global noise to 0.1
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity", 0); // no gravity

    Playground* playground1 = new Playground(odeHandle, osgHandle, osg::Vec3(20.5, 0.2, 2.0),0.05);
    playground1->setColor(Color(0,0.8,0,0.2));
    playground1->setTexture("Images/really_white.rgb");
    playground1->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground1);
    
    Playground* playground2 = new Playground(odeHandle, osgHandle, osg::Vec3(20.5, 0.2, 2.0),0.05);
    playground2->setColor(Color(0,0.8,0,0.2));
    playground2->setTexture("Images/really_white.rgb");
    playground2->setPosition(osg::Vec3(0,1.4,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground2);

//     Box* box = new Box(3, 2.6 ,0.1);
//     box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
//     box->setPose(osg::Matrix::rotate(-M_PI/6,osg::Vec3(0,1,0)) * osg::Matrix::translate(9.0,0.7,0.4));
//     box->update();
//     box = new Box(3, 2.6 ,0.1);
//     box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
//     box->setPose(osg::Matrix::rotate(M_PI/6,osg::Vec3(0,1,0)) * osg::Matrix::translate(-9.0,0.7,0.4));
//     box->update();

    ForcedSphereConf conf;
    ForcedSphere* sphere1;
    ForcedSphere* sphere2;
    AbstractWiring* wiring;
    OdeAgent* agent;

    //////// AGENT 1

    conf = ForcedSphere::getDefaultConf(); 
    //    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis));
    RelativePositionSensor* s = new RelativePositionSensor(4,1,RelativePositionSensor::X);
    s->setReference(playground1->getMainPrimitive());
    conf.addSensor(s);    
    conf.radius = 0.5;
    conf.drivenDimensions = ForcedSphere::X;
    sphere1 = new ForcedSphere ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)), 
				 conf, "Agent1");     
    ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.5 ));

    //    controller = new InvertMotorSpace(50);  
    controller = new InvertMotorNStep();  
    controller->setParam("epsA",0.005); // model learning rate
    controller->setParam("epsC",0.02); // controller learning rate
    //    controller->setParam("rootE",3);    // model and contoller learn with square rooted error
    controller->setParam("factorB",0); 
    controller->setParam("noiseB",0);  
    controller->setParam("adaptrate",0.0); 
    controller->setParam("noiseY",0.0); 
    global.configs.push_back ( controller );

    //controller = new SineController();
    //global.configs.push_back ( controller );
      

    // wiring = new One2OneWiring ( new ColorUniformNoise() );
    DerivativeWiringConf wc = DerivativeWiring::getDefaultConf();
    wc.useId=false;
    wc.useSecondD=true;
    wc.eps=1;
    wc.derivativeScale=100;
    wiring = new DerivativeWiring ( wc, new ColorUniformNoise());
    agent = new OdeAgent ( plotoptions );
    agent->init ( controller , sphere1 , wiring );
    global.agents.push_back ( agent );


    //////// AGENT 2

//     conf = ForcedSphere::getDefaultConf(); 
//     //    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::OnlyZAxis));
//     s = new RelativePositionSensor(4,1,RelativePositionSensor::X);
//     s->setReference(playground2->getMainPrimitive());
//     conf.addSensor(s);    
//     conf.radius = 0.5;
//     conf.drivenDimensions = ForcedSphere::X;
//     sphere2 = new ForcedSphere ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)), 
// 				 conf, "Agent2");     
//     ((OdeRobot*)sphere2)->place ( Pos( 0 , 1.4 , 0.5 ));

//     controller = new InvertMotorSpace(50);  
//     controller->setParam("epsA",0.05); // model learning rate
//     controller->setParam("epsC",0.2); // controller learning rate
//     //    controller->setParam("rootE",3);    // model and contoller learn with square rooted error
//     global.configs.push_back ( controller );

//     // wiring = new One2OneWiring ( new ColorUniformNoise() );    
//     wiring = new DerivativeWiring ( wc, new ColorUniformNoise());
//     agent = new OdeAgent (std::list<PlotOption>());
//     agent->init ( controller , sphere2 , wiring );
//     global.agents.push_back ( agent );
      
    showParams(global.configs);
  }

  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, 
		       int key, bool down) { 
    if (down) { // only when key is pressed, not when released
      switch ( (char) key ) {
//       case 'X' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
//       case 'x' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
//       case 'T' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , 3 ); break;
//       case 't' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , -3 ); break;
//       case 'S' : controller->setParam("sineRate", controller->getParam("sineRate")*1.2); 
// 	printf("sineRate : %g\n", controller->getParam("sineRate"));
//       break;
//       case 's' : controller->setParam("sineRate", controller->getParam("sineRate")/1.2); 
// 	printf("sineRate : %g\n", controller->getParam("sineRate"));
// 	break;      
      default:
	return false;	
      }
      return true;
    } else return false;
  }  

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
//     au.addKeyboardMouseBinding("Simulation: X","Push robot to right (positive x)");
//     au.addKeyboardMouseBinding("Simulation: x","Push robot to left (negative x)");
//     au.addKeyboardMouseBinding("Simulation: T","Spin robot counter-clockwise");
//     au.addKeyboardMouseBinding("Simulation: t","Spin robot clockwise");
    //    au.addKeyboardMouseBinding("Controller: S","Increase sine frequency");
    //    au.addKeyboardMouseBinding("Controller: s","Decrease sine frequency");
  }

};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
 
 
