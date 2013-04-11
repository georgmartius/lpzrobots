/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
 *    mai00bvz@studserv.uni-leipzig.de                                     *
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
 *   Revision 1.7  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.6  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.5  2009/08/05 23:25:57  martius
 *   adapted small things to compile with changed Plotoptions
 *
 *   Revision 1.4  2009/04/02 12:21:46  fhesse
 *   box.setTexture() before setPosition() (due to new OsgBoxTex)
 *
 *   Revision 1.3  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.2  2007/10/10 06:41:10  fhesse
 *   z argument in robots place() call increased
 *   (before it was inside earth (but therefore it worked))
 *
 *   Revision 1.1  2007/10/08 20:17:14  fhesse
 *   initial version of layered controller
 *   and nimm2_layered as testbed for this controller
 *
 *   Revision 1.1  2007/10/06 10:12:02  fhesse
 *   initial version of setup to test homeokinese equipped with hebbian learning
 *   (using invernnchannelcontrollerhebbh)
 *
 *
 ***************************************************************************/
#include <stdio.h>

// include ode library
#include <ode-dbl/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

// used robot
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>


// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>


// used controller
#include "layeredcontroller.h"
//#include <selforg/invertnchannelcontroller.h>
//#include <selforg/invertnchannelcontrollerhebbxsi.h>
//#include <selforg/invertnchannelcontrollerhebbh.h>
//#include <selforg/invertmotorspace.h>
//#include <selforg/sinecontroller.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // first: position(x,y,z) second: view(alpha,beta,gamma)
    // gamma=0;
    // alpha == horizontal angle
    // beta == vertical angle
    setCameraHomePos(Pos(-0.18, 20.36, 13.63), Pos(-179.93, -34.37, 0));

    // initialization
    // - set noise to 0.1
    global.odeConfig.noise=0.1;
    global.odeConfig.setParam("controlinterval", 10);

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)


    bool labyrint=false;
    bool squarecorridor=false;
    bool normalplayground=true;
    bool boxes = true;

    if(normalplayground){
      //     Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(12, 0.2, 0.5));
      Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(17, 0.2, 1.0));
      playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
      // register playground in obstacles list
      global.obstacles.push_back(playground);
    }

    if(squarecorridor){
      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(15, 0.2, 1.2 ), 1);
      playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
      playground->setGroundTexture("Images/really_white.rgb");
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.1));
      playground->setPosition(osg::Vec3(0,0,0.1));
      playground->setTexture("");
      global.obstacles.push_back(playground);
      //     // inner playground
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(10, 0.2, 1.2), 1, false);
      playground->setColor(Color(255/255.0,200/255.0,0/255.0, 0.1));
      playground->setPosition(osg::Vec3(0,0,0.1));
      playground->setTexture("");
      global.obstacles.push_back(playground);
    }

    if(labyrint){
      double radius=7.5;
      Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(radius*2+1, 0.2, 5 ), 1);
      playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
      playground->setGroundTexture("Images/really_white.rgb");
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.1));
      playground->setPosition(osg::Vec3(0,0,0.1));
      playground->setTexture("");
      global.obstacles.push_back(playground);
      int obstanz=30;
      OsgHandle rotOsgHandle = osgHandle.changeColor(Color(255/255.0, 47/255.0,0/255.0));
      OsgHandle gruenOsgHandle = osgHandle.changeColor(Color(0,1,0));
      for(int i=0; i<obstanz; i++){
        PassiveBox* s = new PassiveBox(odeHandle, (i%2)==0 ? rotOsgHandle : gruenOsgHandle,
                                       osg::Vec3(random_minusone_to_one(0)+1.2,
                                                 random_minusone_to_one(0)+1.2 ,1),5);
        s->setPose(osg::Matrix::translate(radius/(obstanz+10)*(i+10),0,i)
                   * osg::Matrix::rotate(2*M_PI/obstanz*i,0,0,1));
        global.obstacles.push_back(s);
      }
    }

    if(boxes) {
      for (int i=0; i<= 2; i+=2){
        PassiveBox* s1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1,1,1), 0.4);
        s1->setTexture("Images/dusty.rgb");
        s1->setPosition(osg::Vec3(-5+i*5,0,0));
        global.obstacles.push_back(s1);
        Joint* fixator;
        Primitive* p = s1->getMainPrimitive();
        fixator = new FixedJoint(p, global.environment);
        fixator->init(odeHandle, osgHandle);

        s1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1,1,1), 0.4);
        s1->setTexture("Images/dusty.rgb");
        s1->setPosition(osg::Vec3(0,-5+i*5,0));
        global.obstacles.push_back(s1);
        p = s1->getMainPrimitive();
        fixator = new FixedJoint(p, global.environment);
        fixator->init(odeHandle, osgHandle);

        s1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1,1,1), 0.4);
        s1->setTexture("Images/dusty.rgb");
        s1->setPosition(osg::Vec3(-3.5+i*3.5,-3.5+i*3.5,0));
        global.obstacles.push_back(s1);
        p = s1->getMainPrimitive();
        fixator = new FixedJoint(p, global.environment);
        fixator->init(odeHandle, osgHandle);

        s1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(1,1,1), 0.4);
        s1->setTexture("Images/dusty.rgb");
        s1->setPosition(osg::Vec3(-3.5+i*3.5,3.5-i*3.5,0));
        global.obstacles.push_back(s1);
        p = s1->getMainPrimitive();
        fixator = new FixedJoint(p, global.environment);
        fixator->init(odeHandle, osgHandle);
      }
    }


    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere
    // - set a texture for the sphere
    // - add sphere to list of obstacles
    for (int i=0; i < 0/*2*/; i++){
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
      s1->setPosition(osg::Vec3(-4.5+i*4.5,0,0));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
    }

    // use Nimm2 vehicle as robot:
    // - get default configuration for nimm2
    // - activate bumpers, cigar mode and infrared front sensors of the nimm2 robot
    // - create pointer to nimm2 (with odeHandle, osg Handle and configuration)
    // - place robot
    Nimm2Conf c = Nimm2::getDefaultConf();
    c.bumper  = false;//true;
    c.cigarMode  = false;//true;
        c.irFront = true;
        c.irBack = true;
        //c.irSide = true;
        c.irRange = 1.2;//2;//3;
    c.force=2;
    c.speed=8;
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, c, "Nimm2");
    vehicle->place(Pos(0,0,0.5));
    //    vehicle->place(Pos(0,6.25,0));


    // use Nimm4 vehicle as robot:
    // - create pointer to nimm4 (with odeHandle and osg Handle and possible other settings, see nimm4.h)
    // - place robot
    //OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle, "Nimm4");
    //vehicle->place(Pos(0,1,0));


    // create pointer to controller
    // push controller in global list of configurables
    AbstractController *controller = new LayeredController(10);
    controller->setParam("eps",0.1);
    controller->setParam("factor_a",0.1);
    controller->setParam("eps_hebb",0.03);
    global.configs.push_back(controller);

    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    agent->setTrackOptions(TrackRobot(true, false, false, false, "hebbh" ,1));
    global.agents.push_back(agent);


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

