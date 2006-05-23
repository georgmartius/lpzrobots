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
 *   Revision 1.15.4.3  2006-05-23 21:57:28  martius
 *   new system
 *
 *   Revision 1.15.4.2  2005/11/16 11:27:38  martius
 *   invertmotornstep has configuration
 *
 *   Revision 1.15.4.1  2005/11/15 12:29:56  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.15  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include "simulation.h"

// include agent (class for holding a robot, a controller and a wiring)
#include "odeagent.h"

// used wiring
#include <selforg/one2onewiring.h>

// used robot
#include "sphererobot3masses.h"

// used arena
#include "playground.h"
#include "meshground.h"
// used passive spheres
#include "passivesphere.h"

// used controller
//#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


Sphererobot3Masses* sphere ;
//const double height = 6.5;
const double height = 3;


class ThisSim : public Simulation {
public:


  void addRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int i){
    Color col;
    
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.diameter=1.0;
    conf.pendularrange= 0.35; // 0.15;
    //SphererobotArms* sphere = new SphererobotArms ( odeHandle, conf);
    if(i==0){
      col.r()=0;
      col.g()=0.5;
      col.b()=1;
      sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere", 0.4);
      sphere->place ( osg::Matrix::translate(9.5 , 0 , height+1 ));
    }else
      if(i==1){
	col.r()=1;
	col.g()=0.4;
	col.b()=0;
	sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere", 0.4);
	sphere->place ( osg::Matrix::translate( 2 , -2 , height+1 ));
      } else {
	col.r()=0;
	col.g()=1;
	col.b()=0.4;
	sphere = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(col), conf, "sphere", 0.4);
	sphere->place ( osg::Matrix::translate( double(rand())/RAND_MAX*10 , 0 , height+1 ));
      }
  
  
    //AbstractController *controller = new InvertNChannelController(10);  
    AbstractController *controller = new InvertMotorNStep();
    //    controller->setParam("factorB", 0.1);
    controller->setParam("steps", 2);
    //    controller->setParam("nomupdate", 0.005);
  
    AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
    OdeAgent* agent = new OdeAgent ( i==0 ? plotoptions : list<PlotOption>() );
    agent->init ( controller , sphere , wiring );
    global.agents.push_back ( agent );
    global.configs.push_back ( controller );  
  }

  void removeRobot(GlobalData& global){
    if(!global.agents.empty()){
      OdeAgentList::iterator i =  global.agents.end()-1;
      delete (*i)->getRobot();
      delete (*i)->getController(); 
      delete (*i);
      global.agents.erase(i);    
    }
  }


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
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
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(10, 0.2, 0.5));
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);


    MeshGround* meshground = new MeshGround(odeHandle, osgHandle,"mesh.dat");
    meshground->setPose(osg::Matrix::translate(0,0,0));
    global.obstacles.push_back(meshground);
    
    addRobot(odeHandle, osgHandle, global, 0);
    addRobot(odeHandle, osgHandle, global, 1);

    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and 
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere 
    // - set a texture for the sphere
    // - add sphere to list of obstacles
    for (int i=0; i<= 1/*2*/; i+=2){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
      s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }
  
    showParams(global.configs);
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
  return sim.run(argc, argv) ? 0 : 1;

}
 




