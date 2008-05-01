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
 *   Revision 1.2  2008-05-01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2006/10/05 11:41:23  robot8
 *   -modular nimm4 robot
 *   -constructed from components
 *   -wheels could be removed by keyboard commands within simulation
 *
 *   Revision 1.7  2006/08/04 16:25:14  martius
 *   bugfixing
 *
 *   Revision 1.6  2006/07/14 12:23:50  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.5.4.5  2006/06/25 21:57:41  martius
 *   robot names with numbers
 *
 *   Revision 1.5.4.4  2006/06/25 17:01:55  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.5.4.3  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.5.4.2  2006/01/18 16:46:56  martius
 *   moved to osg
 *
 *   Revision 1.1.2.3  2006/01/17 17:02:47  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2006/01/13 12:33:16  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.1  2006/01/13 12:24:06  martius
 *   env for external teaching input to the controller
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <vector.h>
#include <osg/Matrix>

#include <ode_robots/simplecomponent.h>
#include <ode_robots/component.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


AbstractController* controller;
motor teaching[2];

SimpleComponent* body;
SimpleComponent* wheel[4];


class ThisSim : public Simulation
{

public:
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {

    setCameraHomePos ( Pos ( 0, 0, 7), Pos ( -0.0828247 , -89.9146 , 0) );
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.1;
    //  global.odeConfig.setParam("gravity", 0);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the 
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground: 
    //   setGeometry(double length, double width, double	height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)

    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(11, 0.2, 1), 12);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);
    //****************

    ComponentConf cConf = Component::getDefaultConf ();
    cConf.max_force = 2;
    cConf.speed = 10;

//adding the controller for the component-connections

    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();

    cc.cInit=1.2;
    AbstractController* controller;
    DerivativeWiringConf c = DerivativeWiring::getDefaultConf ();
    DerivativeWiring* wiring;
    OdeAgent* agent;

    body = new SimpleComponent ( odeHandle , osgHandle , cConf );
    wheel[0] = new SimpleComponent ( odeHandle , osgHandle , cConf );
    wheel[1] = new SimpleComponent ( odeHandle , osgHandle , cConf );
    wheel[2] = new SimpleComponent ( odeHandle , osgHandle , cConf );
    wheel[3] = new SimpleComponent ( odeHandle , osgHandle , cConf );
    
    body->setSimplePrimitive ( new Capsule ( 0.1 , 1 ) );
    body->getMainPrimitive ()->init ( odeHandle , 0.5 , osgHandle.changeColor ( Color ( 2, 156/255.0, 0, 1.0f ) ) );
    body->getMainPrimitive()->getOSGPrimitive()->setTexture("Images/wood.rgb");

    for ( int n = 0; n < 4; n++ )
    {
	wheel[n]->setSimplePrimitive ( (Primitive*) new Sphere ( 0.3 ) );
	wheel[n]->getMainPrimitive ()->init ( odeHandle , 0.2 , osgHandle.changeColor ( Color ( 0.8,0.8,0.8 )) );
	wheel[n]->getMainPrimitive()->getOSGPrimitive()->setTexture("Images/wood.rgb");
    } 


    body->place ( osg::Matrix::rotate ( M_PI/2 , osg::Vec3 (1 , 0 , 0 ))*osg::Matrix::translate ( 0 , 0 , 0.5 ) );

    wheel[0]->place ( Pos ( -0.45 , -0.5 , 0.5 ) );
    wheel[1]->place ( Pos ( 0.45 , -0.5 , 0.5 ) );
    wheel[2]->place ( Pos ( -0.45 , 0.5 , 0.5 ) );
    wheel[3]->place ( Pos ( 0.45 , 0.5 , 0.5 ) );

    for ( int n = 0; n < 4; n++ )
    {
	Axis axis = Axis ( ( wheel[n<2?0:2]->getPosition () - wheel[n<2?1:3]->getPosition ()).toArray() );
	HingeJoint* j1 = new HingeJoint ( body->getMainPrimitive () , wheel[n]->getMainPrimitive () , osg::Vec3 (0,0.5*( n<2?-1:1),0.5) , axis );
	j1->init ( odeHandle , osgHandle , true , 1 );

	body->addSubcomponent ( wheel[n] , j1 , false );
    } 

	    //controller
	    controller = new InvertMotorNStep ( cc );
	    controller->setParam ("adaptrate", 0.0);
	    controller->setParam ("epsC", 0.1);
	    controller->setParam ("epsA", 0.01);
	    controller->setParam ("rootE", 3);
	    controller->setParam ("steps", 2);
	    controller->setParam ("s4avg", 2);
	    controller->setParam ("factorB",0);
//	    controller = new SineController ( 18 );
	    //wiring
	    wiring = new DerivativeWiring ( c , new ColorUniformNoise () );   
	    //agent
	    agent = new OdeAgent ( plotoptions );
	    agent->init ( controller , body , wiring );
	    global.agents.push_back ( agent );
	    global.configs.push_back ( controller );   

    showParams(global.configs);
  }   
    

  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;    
    bool handled = false;
    FILE* f;
    switch ( key )
      {
      case 's' :
	f=fopen("controller","wb");
	controller->store(f) && printf("Controller stored\n");
	fclose(f);
	handled = true; break;	
      case 'l' :
	f=fopen("controller","rb");
	controller->restore(f) && printf("Controller loaded\n");
	handled = true; break;	
	fclose(f);
      case 'A':
	  body->removeSubcomponent ( wheel[0] );
	  break;
      case 'S':
	  body->removeSubcomponent ( wheel[1] );
	  break;
      case 'Q':
	  body->removeSubcomponent ( wheel[2] );
	  break;
      case 'W':
	  body->removeSubcomponent ( wheel[3] );
	  break;



      }
    fflush(stdout);
    return handled;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Teachung: t","toggle mode");
    au.addKeyboardMouseBinding("Teaching: u","forward");
    au.addKeyboardMouseBinding("Teaching: j","backward");
    au.addKeyboardMouseBinding("Simulation: s","store");
    au.addKeyboardMouseBinding("Simulation: l","load");
  }

  
};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
 
