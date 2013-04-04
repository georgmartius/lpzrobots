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
 *   Revision 1.5  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.4  2007/12/12 15:39:25  robot3
 *   Gwizdziel
 *
 *   Revision 1.3  2007/12/11 16:33:06  robot3
 *   *** empty log message ***
 *
 *   Revision 1.2  2007/12/10 15:39:41  robot3
 *   Gwizdziel
 *
 *   Revision 1.1  2007/01/18 15:40:15  robot3
 *   special version for master thesis
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

// used robot
#include <ode_robots/barrel2masses2nd.h>

//server
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


// used arena
#include <ode_robots/playground.h>

// used controller
#include <selforg/sinecontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/use_java_controller.h>



//int use_java_controller::anzahl_Java_controller = 2;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class ThisSim : public Simulation
{
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {

    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    global.odeConfig.noise=0.05;

    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(10, 0.2, 0.5),1,true);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren

    global.obstacles.push_back(playground);


    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
    conf.motorsensor=true;
    conf.irAxis1=true;
    conf.irAxis2=true;
    conf.irAxis3=true;
    conf.irRing=false;
    conf.irSide=false;
    conf.pendularrange  = .4;



    // zusaetzlich das rauschen, gibt weisses und color...
    One2OneWiring* wiring1 = new One2OneWiring ( new ColorUniformNoise() );
    One2OneWiring* wiring2 = new One2OneWiring ( new ColorUniformNoise() );


    //Kugel 1 ***********************************************

    Sphererobot3Masses* myBarrel_1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,1.0)),conf, "Rocco-Kugel-1-Java-Controller");
    ((OdeRobot*)myBarrel_1)->place (Pos( 1.3,0,0.1));

    //Controller Kugel 1

        use_java_controller* jcontroller1;
    jcontroller1 = new use_java_controller("4811","4812","robot2");

    OdeAgent* agent_1 = new OdeAgent( global, plotoptions );
    agent_1->init ( jcontroller1 , myBarrel_1 , wiring1 );
    global.agents.push_back ( agent_1 );
    global.configs.push_back ( jcontroller1 );

    // ENDE****************************************************




    //Kugel 2 ***********************************************
    Sphererobot3Masses* myBarrel_2 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)),conf, "Rocco-Kugel-2-Java-Controller");
    myBarrel_2->place ( osg::Matrix::rotate(M_PI/2, 1,0,0));

    //Controller Kugel 2

    use_java_controller* Jcontroller;
    //SineController* Jcontroller;
   Jcontroller = new use_java_controller("4711","4712","robot1");
   //Jcontroller = new SineController();

    OdeAgent* agent_2 = new OdeAgent( global, plotoptions);
    agent_2->init ( Jcontroller , myBarrel_2 , wiring2);
    global.agents.push_back ( agent_2 );
    global.configs.push_back ( Jcontroller );
    // ENDE****************************************************



  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down)
    { // only when key is pressed, not when released
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
  ThisSim simu;
  simu.run(argc, argv);

}

