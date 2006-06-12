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
 *   Revision 1.1.2.1  2006-06-12 12:59:08  robot8
 *   -some corrections to component system (for example now connections are only pushed as pointers to the connection vector)
 *   -created the evolution simulation for evolutionary evolution algorithms with atom like structures
 *
 *
 ***************************************************************************/

#include "simulation.h"

#include "odeagent.h"
#include "atomodeagent.h"
#include "octaplayground.h"
#include "passivesphere.h"

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include "sphererobot.h"
#include "schlangeservo.h"
#include "sphererobot3masses.h"

#include "component.h"
#include "simplecomponent.h"
#include "robotcomponent.h"
#include "atomcomponent.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;



class ThisSim : public Simulation 
{

public:

    vector <Component*> components;

public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(3, 3, 3), Pos(140.539, -13.1456, 0));
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
    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 5), 4);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    //****************

    
//atoms

    ComponentConf cConf = Component::getDefaultConf ();
    AtomConf aConf = AtomComponent::getDefaultAtomConf ();
    cConf.speed = 5;
    cConf.max_force = 5;

//    aConf.shell_radius = 0.2;
    aConf.max_bindings = 4;
    aConf.binding_energy = 4;

    components.push_back ( new AtomComponent ( odeHandle , osgHandle , cConf , aConf ) );
//    components.push_back ( new Component ( odeHandle , osgHandle , cConf ) );
    components.back ()->place ( Pos( 0 , 0 , aConf.shell_radius*2 )); 

//wirings
    DerivativeWiring* wiring;
    AtomOdeAgent* agent;

//unbound atoms
    for ( int m = 1; m <= 10; m++ )
    {
	components.push_back ( new AtomComponent ( odeHandle , osgHandle , cConf , aConf ) );
	components.back ()->place ( Pos( 0.05*m , 0 , m*2 )); 
    }





//adding the controller for the component-connections
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.cInit=2;


    AbstractController* controller = new InvertMotorNStep ( cc );
    controller->setParam("adaptrate", 0.005);
    controller->setParam("epsC", 0.005);
    controller->setParam("epsA", 0.001);
    controller->setParam("rootE", 3);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 5);
    controller->setParam("factorB",0);

    DerivativeWiringConf c = DerivativeWiring::getDefaultConf ();
    wiring = new DerivativeWiring ( c , new ColorUniformNoise() );   
    


    agent = new AtomOdeAgent ( plotoptions );
    agent->init ( controller , components.front () , wiring );
    global.agents.push_back ( agent );
    global.configs.push_back ( controller );




    showParams(global.configs);
  } 

    void addCallback ( GlobalData& globalData , bool draw , bool pause )
	{
	    for ( unsigned int n = 0; n < components.size (); n++ )
		components[n]->update (); //not realy perfect, because the atoms belonging to robots are drawn an additional time by the agents
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
