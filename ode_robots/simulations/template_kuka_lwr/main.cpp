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
#include <ode_robots/kuka.h>


// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>

// used controller
#include <selforg/invertmotorspace.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
	Kuka* arm;
	std::vector<Primitive*> objectsOfInterest;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // first: position(x,y,z) second: view(alpha,beta,gamma)
    // gamma=0;
    // alpha == horizontal angle
    // beta == vertical angle
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set noise to 0.1
    global.odeConfig.noise=0.05;
    //  global.odeConfig.setParam("gravity", 0);

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground:
    //   setGeometry(double length, double width, double	height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)

    // odeHandle and osgHandle are global references
    // vec3 == length, width, height
    Playground* playground = new Playground(odeHandle, osgHandle, osg::Vec3(32, 0.2, 0.5));
    playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
    // register playground in obstacles list
    global.obstacles.push_back(playground);

	//This is a list of objects the Kuka Arm will keep track of by means of relative position sensors
	objectsOfInterest.resize(1);
    
    // add passive sphere as obstacle
    // - create pointer to sphere (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere
    // - set a texture for the sphere
    // - add sphere to list of obstacles
    // - add sphere to list of objectsof Interest
  
      PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.1);
      s1->setPosition(osg::Vec3(1,0,0));
      s1->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s1);
      objectsOfInterest[0] = s1->getMainPrimitive();
  
    // use Kuka Arm as robot:
    // - create pointer to Kuka Arm (with odeHandle and osg Handle, the size and the objects Of Interest)
    // - place robot		
    OdeRobot* vehicle = new Kuka(odeHandle, osgHandle, "Kuka", 2, objectsOfInterest);
    vehicle->place(Pos(0,0,0));
	arm = (Kuka*) vehicle;

	//fix arm to ground
	Joint* fixator;
	Primitive* socket = arm->getMainPrimitive();
    	fixator = new FixedJoint(socket, global.environment);
        fixator->init(odeHandle, osgHandle);
   	fixator->setParam(dParamHiStop, 0);
    	fixator->setParam(dParamLoStop, 0);

    // create pointer to controller
    // push controller in global list of configurables
      //AbstractController *controller = new KukaController();
	AbstractController *controller = new InvertMotorSpace(10);
    //global.configs.push_back(controller);

    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    // create pointer to agent
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, arm, wiring);
    global.agents.push_back(agent);

  }

/* add own key handling stuff here, just insert some case values
	l q w e r t z 	move joint indivdually in positive direction
	L Q W E R T Z 	move joint indivdually in negative direction
	i 		go to initial candle position
	j 		print joint configuration
	g/G 		grasp/release the ball (if close enough)
	o/O		add/remove random Object (very funny:)
	h		show onscreen help and more options
*/
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
 case 'm':
	arm->toggleManualControlMode();
	break;
	
case 'l':
        { double posj= arm->getJointTarget(0);
        //std::cout << "Move joint 0 from" << posj << " to " << posj + 0.1 << "\n";
        arm->moveJoint(0,  posj + 0.1754);
        break;
        }
    case 'L':
        { double posj= arm->getJointTarget(0);
        //std::cout << "Move joint 0 from" << posj << " to " << posj - 0.1 << "\n";
        arm->moveJoint(0, posj - 0.1754);
        break;}
    case 'q':
        { double posj= arm->getJointTarget(1);
        //std::cout << "Move joint 1 from" << posj << " to " << posj + 0.1 << "\n";
        arm->moveJoint(1, posj + 0.1754);
        break;}
    case 'Q':
        { double posj= arm->getJointTarget(1);
        //std::cout << "Move joint 1 from" << posj << " to " << posj - 0.1 << "\n";
        arm->moveJoint(1, posj - 0.1754);
        break;}
    case 'w':
        { double posj= arm->getJointTarget(2);
        //std::cout << "Move joint 2 from" << posj << " to " << posj + 0.1 << "\n";
        arm->moveJoint(2, posj + 0.1754);
        break;}
    case 'W':
        { double posj= arm->getJointTarget(2);
        //std::cout << "Move joint 2 from" << posj << " to " << posj - 0.1 << "\n";
        arm->moveJoint(2, posj - 0.1754);
        break;}
    case 'e':
        { double posj= arm->getJointTarget(3);
        //std::cout << "Move joint 3 from" << posj << " to " << posj + 0.1 << "\n";
        arm->moveJoint(3, posj + 0.1754);
        break;}
    case 'E':
        { double posj= arm->getJointTarget(3);
        //std::cout << "Move joint 3 from" << posj << " to " << posj - 0.1 << "\n";
        arm->moveJoint(3, posj - 0.1754);
        break;}
    case 'r':
        { double posj= arm->getJointTarget(4);
        //std::cout << "Move joint 4 from" << posj << " to " << posj + 0.1 << "\n";
        arm->moveJoint(4, posj + 0.1754);
        break;}
    case 'R':
        { double posj= arm->getJointTarget(4);
       // std::cout << "Move joint 4 from" << posj << " to " << posj - 0.1 << "\n";
        arm->moveJoint(4, posj - 0.1754);
        break;}
    case 't':
        { double posj= arm->getJointTarget(5);
        //std::cout << "Move joint 5 from" << posj << " to " << posj + 0.1 << "\n";
        arm->moveJoint(5, posj + 0.1754);
        break;}
    case 'T':
        { double posj= arm->getJointTarget(5);
        //std::cout << "Move joint 5 from" << posj << " to " << posj - 0.1 << "\n";
        arm->moveJoint(5, posj - 0.1754);
        break;}
    case 'z':
        { double posj= arm->getJointTarget(6);
        //std::cout << "Move joint 6 from" << posj << " to " << posj + 0.1 << "\n";
        arm->moveJoint(6, posj + 0.1754);
        break;}
    case 'Z':
	{
         double posj= arm->getJointTarget(6);
       // std::cout << "Move joint 6 from" << posj << " to " << posj - 0.1 << "\n";
        arm->moveJoint(6, posj - 0.1754);
        break;}
    case 'g':
	{
	objectsOfInterest[0]->getPosition();
	arm->grasp(objectsOfInterest[0]);
	break;
	}
    case 'G': 
	{arm->release();
	break;
	}
    case 'i':
        arm->candle();
        break;
    case 'j':
        arm->printJointConf();
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
