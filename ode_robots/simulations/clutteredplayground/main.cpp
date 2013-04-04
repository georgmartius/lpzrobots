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
 *   Revision 1.8  2011-10-14 09:36:18  martius
 *   snakes have no frictionGround parameter anymore, since it was not used,
 *    use the substances now
 *
 *   Revision 1.7  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.6  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.5  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.4  2007/01/26 12:07:08  martius
 *   orientationsensor added
 *
 *   Revision 1.3  2006/08/11 15:46:34  martius
 *   *** empty log message ***
 *
 *   Revision 1.2  2006/07/14 12:23:45  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/05/23 13:43:19  robot3
 *   first version
 *
 *   Revision 1.14.4.15  2006/05/18 11:55:56  robot3
 *   made playground smaller (for shadowing issues)
 *
 *   Revision 1.14.4.14  2006/05/15 13:11:30  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.14.4.13  2006/04/25 09:06:16  robot3
 *   *** empty log message ***
 *
 *   Revision 1.14.4.12  2006/03/31 12:14:49  fhesse
 *   orange color for nimm robot
 *
 *   Revision 1.14.4.11  2006/03/31 11:27:53  fhesse
 *   documentation updated
 *   one sphere removed (todo  fix problem with sphere placing)
 *
 *   Revision 1.14.4.10  2006/01/12 15:17:46  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.9  2005/12/29 15:55:33  martius
 *   end is obsolete
 *
 *   Revision 1.14.4.8  2005/12/29 15:47:12  martius
 *   changed to real Sim class
 *
 *   Revision 1.14.4.7  2005/12/14 15:37:25  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.6  2005/12/13 18:12:09  martius
 *   switched to nimm2
 *
 *   Revision 1.14.4.5  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.4  2005/12/09 16:53:17  martius
 *   camera is working now
 *
 *   Revision 1.14.4.3  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *   Revision 1.14.4.2  2005/11/24 16:19:12  fhesse
 *   include corrected
 *
 *   Revision 1.14.4.1  2005/11/15 12:30:07  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
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
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/schlangeservo2.h>

// used arena
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>

// used passive spheres
#include <ode_robots/passivesphere.h>
// used passive boxes
#include <ode_robots/passivebox.h>
// used passive capsules
#include <ode_robots/passivecapsule.h>

// used controller
#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>

// use sin and cos
#include <math.h>



// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(6.5701, 17.3534, 11.7749),  Pos(159.449, -30.0839, 0));
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
    //   setGeometry(double length, double width, double        height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
    OctaPlayground* playground =
      new OctaPlayground(odeHandle, osgHandle,
                         osg::Vec3(8.0f, 0.2, 6.0f),
                         12, // number of corners
                         true); // if ground is created (only create one)
    // true); // if ground is created (only create one)
    // when you not set this texture, a brown wooden texture
    // is used, the setColor would overlay with the wooden texture
    playground->setColor(Color(1.0,0.5,0.0,0.2));
    // when you not set this texture, a brown wooden texture
    // is used, the setColor would overlay with the wooden texture
    playground->setTexture("Images/really_white.rgb");
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);



    OctaPlayground* playground2 =
      new OctaPlayground(odeHandle, osgHandle,
                         osg::Vec3(1.0f, 0.2, 2.5f),
                         12, // number of corners
                         false); // if ground is created (only create one)
    playground2->setColor(Color(1.0,0.5,0.0,0.1));
    // when you not set this texture, a brown wooden texture
    // is used, the setColor would overlay with the wooden texture
    playground2->setTexture("Images/really_white.rgb");
    playground2->setPosition(osg::Vec3(0.0,0.0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground2);


    // Creation of passive boxes
    // add passive spheres, boxes and capsules as obstacles
    // - create pointer to sphere, box or capsule (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere, box or capsule
    // - set a texture for the sphere, box or capsule
    // - add sphere, box or capsule to list of obstacles
    // note that the dependent header file has to be include above
    // ("passivesphere.h", "passivebox.h" and/or "passivecapsule.h"

    int n=10;
    int m=3;
    for (int j=0;j<m;j++) {
      for(int i=0; i<n; i++){
        PassiveSphere* s =
          new PassiveSphere(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)),
                          /*Diameter=*/.2+i*0.03,/*Mass=*/.5);
        s->setPosition(Pos(sin(2*M_PI*(((float)i)/((float)n)+0.05f))*(3.0f+2.0f*j),
                           cos(2*M_PI*(((float)i)/((float)n)+0.05f))*(3.0f+2.0f*j),
                           0.8f+2.0f*j));
        s->setTexture("Images/dusty.rgb");
        global.obstacles.push_back(s);
      }

    }
    n=8;
    m=3;
    for (int j=0;j<m;j++) {
      for(int i=0; i<n; i++){
        PassiveBox* b =
          new  PassiveBox(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)),
                          osg::Vec3(0.4+i*0.1, 0.8-i*0.03, 0.4), 0.2+i*0.1);
        if (i==1)
          b->setTexture("Images/light_chess.rgb");
        else
          b->setTexture("Images/dusty.rgb");
        b->setPosition(Pos(sin(2*M_PI*(((float)i)/((float)n))+0.1f)*(3.0f+2.0f*j),
                           cos(2*M_PI*(((float)i)/((float)n))+0.1f)*(3.0f+2.0f*j),
                           3.0f+2.0f*j));
        global.obstacles.push_back(b);
      }

    }
    n=15;
    m=3;
    for (int j=0;j<m;j++) {
      for(int i=0; i<n; i++){
        PassiveCapsule* b =
          new  PassiveCapsule(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)),
                          0.2f+0.03*i,1.0f-i*0.04);
        b->setPosition(Pos(sin(2*M_PI*(((float)i)/((float)n)))*(3.0f+2.0f*j),
                           cos(2*M_PI*(((float)i)/((float)n)))*(3.0f+2.0f*j),
                           5.0f+2.0f*j));
        b->setTexture("Images/dusty.rgb");
        global.obstacles.push_back(b);
      }

    }


 // Creation of spherical robots:
    for(int i=0; i<0; i++){
      OdeRobot* sphere1;
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.diameter=2;
      conf.irAxis1=true;
      conf.irAxis2=true;
      conf.irAxis3=true;
       sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(0.0+.1*i,0.0,1.0)),
                                               conf, "Sphere1", 0.4);
       // sphere1 = new ForcedSphere(odeHandle, osgHandle, "FSphere");

       // sphere1->place ( Pos(-3,1/2,3+2*i));
      sphere1->place ( Pos(0,0,7+2*i));
      AbstractController* controller = new InvertMotorNStep();
      controller->setParam("steps", 2);
      controller->setParam("adaptrate", 0.005);
      controller->setParam("nomupdate", 0.01);
      controller->setParam("epsC", 0.01);
      controller->setParam("epsA", 0.005);
      controller->setParam("rootE", 3);

      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = true;
      c.useFirstD = true;
      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );


       // One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
      OdeAgent* agent = new OdeAgent ( plotoptions );
      agent->init ( controller , sphere1 , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
    }
 //creation of snakes
      for(int i=0; i<1; i++){
      //****************/
      SchlangeConf conf = Schlange::getDefaultConf();
      conf.motorPower=.3;
      conf.segmNumber =10-i/2;
      conf.segmDia    = 0.15;   //  diameter of a snake element

      // conf.jointLimit=conf.jointLimit*3;
      conf.jointLimit=conf.jointLimit*2.0;
      conf.frictionJoint=0.002;
      SchlangeServo2* schlange1 =
        //new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
        new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(1.0, 1.0, 1.0)),
                             conf, "S1");
      //Positionieren und rotieren
      schlange1->place(osg::Matrix::rotate(M_PI/2, 0, 1, 0)*
                       osg::Matrix::translate(2*i,i,conf.segmNumber/2+2));
      if (i==0) {
        //      schlange1->setTexture("Images/whitemetal_tiefgruen.rgb");
        //      schlange1->setHeadTexture("Images/whitemetal_tiefrot.rgb");
      } else {
        //        schlange1->setTexture("Images/whitemetal_tiefrot.rgb");
        //        schlange1->setHeadTexture("Images/whitemetal_tiefgruen.rgb");
      }


      //AbstractController *controller = new InvertNChannelController(100/*,true*/);
      //  AbstractController *controller = new InvertMotorSpace(100/*,true*/);
      AbstractController *controller = new InvertMotorNStep();
      // AbstractController *controller = new invertmotornstep();
      //AbstractController *controller = new SineController();
      //    AbstractController *controller = new InvertMotorNStep();

      //     AbstractController *controller = new SineController();

      //    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); //Only this line for one2Onewiring


      //   AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = true;
      c.useFirstD = true;
      //   c.derivativeScale=10;
      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );



      OdeAgent* agent = new OdeAgent(global);
      agent->init(controller, schlange1, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(schlange1);


      global.odeConfig.setParam("controlinterval",2);
      global.odeConfig.setParam("gravity", -2);

      controller->setParam("steps",2);
      controller->setParam("epsC",0.01);
      controller->setParam("epsA",0.01);
      controller->setParam("adaptrate",0.005);
          controller->setParam("logaE",3);

      // controller->setParam("desens",0.0);
         controller->setParam("s4delay",3.0);
      //   controller->setParam("s4avg",1.0);

         controller->setParam("factorB",0.0);
      //   controller->setParam("zetaupdate",0.1);

      }//creation of snakes End


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

