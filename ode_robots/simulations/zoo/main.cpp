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
 *
 ***************************************************************************/
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>


#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>

#include <selforg/sos.h>
#include <selforg/sox.h>
#include <selforg/soml.h>

#include <ode_robots/axisorientationsensor.h>

#include <ode_robots/hurlingsnake.h>
#include <ode_robots/schlangeservo2.h>
#include <ode_robots/caterpillar.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/hexapod.h>

#include "environment.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// predicate that matches agents that have the same name prefix
struct agent_match_prefix : public unary_function<const OdeAgent*, bool> {
    agent_match_prefix(string nameprefix)
      : nameprefix(nameprefix) {
      len=nameprefix.length();
    }
    bool operator()(const OdeAgent* a) {
      if(!a || !a->getRobot()) return false;
      return nameprefix.compare(0,len, a->getRobot()->getName(),0,len) == 0;
    }

    string nameprefix;
    int len;
  };

class ThisSim : public Simulation {
public:

  Env env;
  double lastRobotCreation;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    int numHexapods   = 0;
    int numSphericals = 1;
    int numSnakes     = 2;

    int numCater=1;
    int numNimm2=0;
    int numNimm4=0;
    int numHurling=0;
    int numSliderWheele=0;

    setCameraHomePos(Pos(-19.15, 13.9, 6.9),  Pos(-126.1, -17.6, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.01);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("realtimefactor",1);

    // environemnt type
    env.type=Env::Normal;
    env.create(odeHandle, osgHandle, global);

    env.numSpheres  = 0; 
    env.numBoxes    = 0;   
    env.numCapsules = 0;
    env.placeObstacles(odeHandle, osgHandle, global);
            
    OdeAgent* agent;
    AbstractWiring* wiring;
    OdeRobot* robot;
    AbstractController *controller;
    
    // So wird das mit allen Robotern aussehen, createXXX, siehe unten
    for(int i=0; i < numHexapods ; i++) { 
      agent=createHexapod(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,1+1*i));
    }    
    for(int r=0; r < numSphericals; r++) {
      agent=createSpherical(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,0+1*r));
    }
    for(int r=0; r < numSnakes; r++) {
      agent=createSnake(odeHandle, osgHandle, global, osg::Matrix::translate(4,4-r,0.2));
    }
    
    


    //******* R A U P E  *********/
    for(int r=0; r < numCater ; r++) { 
      CaterPillar* myCaterPillar;
      CaterPillarConf myCaterPillarConf = DefaultCaterPillar::getDefaultConf();
      myCaterPillarConf.segmNumber=3+r;
      myCaterPillarConf.jointLimit=M_PI/3;
      myCaterPillarConf.motorPower=0.2;
      myCaterPillarConf.frictionJoint=0.01;
      myCaterPillar =
	new CaterPillar ( odeHandle, osgHandle.changeColor(Color(1.0f,0.0,0.0)), 
			  myCaterPillarConf, "Raupe" );//+ std::itos(r));
      ((OdeRobot*) myCaterPillar)->place(Pos(-5,-5+2*r,0.2)); 
      
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit=2.0;
      controller = new InvertMotorSpace(15);
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( global, plotoptions );
      agent->init(controller, myCaterPillar, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(myCaterPillar);   
      myCaterPillar->setParam("gamma",/*gb");
					global.obstacles.push_back(s)0.0000*/ 0.0);
    }
    
   
    //******* N I M M  2 *********/
    Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
    nimm2conf.size = 1.6;
    for(int r=0; r < numNimm2; r++) { 
      robot = new Nimm2(odeHandle, osgHandle, nimm2conf, "Nimm2_" + std::itos(r));
      robot->place(Pos ((r-1)*5,5,0));
      //    controller = new InvertMotorNStep(10);   
      controller = new InvertMotorSpace(15);   
      controller->setParam("s4avg",10);
      //    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( global, PlotOption(NoPlot) );
      agent->init(controller, robot, wiring);
      global.configs.push_back(controller);
      global.agents.push_back(agent);        
    }
    
    //******* N I M M  4 *********/
    for(int r=0; r < numNimm4; r++) {
      robot = new Nimm4(odeHandle, osgHandle, "Nimm4_" + std::itos(r));
      robot->place(Pos((r-1)*5,-3,0));
      controller = new InvertMotorSpace(20);
      controller->setParam("s4avg",10); 
      controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( global, PlotOption(NoPlot) );
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);        
    }

    //****** H U R L I N G **********/
    for(int r=0; r < numHurling; r++) {
      HurlingSnake* snake;
      Color c;    
      if (r==0) c=Color(0.8, 0.8, 0);
      if (r==1) c=Color(0,   0.8, 0);
      snake = new HurlingSnake(odeHandle, osgHandle.changeColor(c), "HurlingSnake_" + std::itos(r));
      ((OdeRobot*) snake)->place(Pos(r*5,-6,0.3));
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit=1.5;
      controller = new InvertMotorNStep(invertnconf);
      controller->setParam("steps", 2);
      controller->setParam("adaptrate", 0.001);
      controller->setParam("nomupdate", 0.001);
      controller->setParam("factorB", 0);
    
      // deriveconf = DerivativeWiring::getDefaultConf();
      //     deriveconf.blindMotorSets=0;
      //     deriveconf.useId = true;
      //     deriveconf.useFirstD = true;
      //     deriveconf.derivativeScale = 50;
      //     wiring = new DerivativeWiring(deriveconf, new ColorUniformNoise(0.1));
      wiring = new One2OneWiring(new ColorUniformNoise(0.05));
      agent = new OdeAgent( global, PlotOption(NoPlot) );
      agent->init(controller, snake, wiring);
			       global.configs.push_back(controller);
			       global.agents.push_back(agent);     
    }


    /******* S L I D E R - w H E E L I E *********/
    SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
    for(int r=0; r < numSliderWheele; r++) {
      mySliderWheelieConf.segmNumber=8;
      //mySliderWheelieConf.jointLimit=M_PI/4;
      mySliderWheelieConf.motorPower=0.4;
      mySliderWheelieConf.frictionGround=0.8;
      mySliderWheelieConf.sliderLength=1;
      mySliderWheelieConf.segmLength=0.4;
      
      SliderWheelie* mySliderWheelie = 
	new SliderWheelie(odeHandle, osgHandle, mySliderWheelieConf, "sliderWheelie" + std::itos(r));
      ((OdeRobot*) mySliderWheelie)->place(Pos(4-2*r,0,0.0)); 
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit=1;
      controller = new InvertMotorNStep(invertnconf);    
      controller->setParam("steps",2);
      controller->setParam("factorB",0);
      
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = false;
      c.useFirstD = true;
      DerivativeWiring* wiring3 = new DerivativeWiring ( c , new ColorUniformNoise(0.1) );
      agent = new OdeAgent(global);
      agent->init(controller, mySliderWheelie, wiring3);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(mySliderWheelie);   
    }

    lastRobotCreation=0;    
  }

  bool removeRobot(GlobalData& global, const string& nameprefix){
    OdeAgentList::reverse_iterator i = 
      find_if(global.agents.rbegin(), global.agents.rend(), agent_match_prefix(nameprefix));
    if(i!=global.agents.rend()){
      printf("Removed robot %s\n", (*i)->getRobot()->getName().c_str());
      OdeAgent* a = *i;
      global.agents.erase(i.base()-1);
      removeElement(global.configs, a);
      delete a;
      return true; 
    }
    return false;
  }
  
  OdeAgent* createHexapod(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                          GlobalData& global, osg::Matrix pose){
    // find robot and do naming
    string name("Hexapod");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    HexapodConf myHexapodConf        = Hexapod::getDefaultConf();
    myHexapodConf.coxaPower          = .8;
    myHexapodConf.tebiaPower         = .5;
    myHexapodConf.coxaJointLimitV    = .9; // M_PI/8;  // angle range for vertical dir. of legs
    myHexapodConf.coxaJointLimitH    = 1.3; //M_PI/4;
    myHexapodConf.tebiaJointLimit    = 1.8; // M_PI/4; // +- 45 degree
    myHexapodConf.percentageBodyMass = .5;
    myHexapodConf.useBigBox          = true;
    myHexapodConf.tarsus             = true;
    myHexapodConf.numTarsusSections  = 1;
    myHexapodConf.useTarsusJoints    = true;
    

    OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance.toRubber(20);    
    OdeRobot* robot = new Hexapod(rodeHandle, osgHandle.changeColor(Color(1,1,1)), 
                                  myHexapodConf, name);
    robot->place(osg::Matrix::rotate(M_PI*0,1,0,0)*pose);
    
    AbstractController* controller = new Sox(1.2, false);
    controller->setParam("epsC",0.3);
    controller->setParam("epsA",0.05);
    controller->setParam("Logarithmic",1);
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent*       agent  = new OdeAgent( global);
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);      
    return agent;
  }

  OdeAgent* createSpherical(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                            GlobalData& global, osg::Matrix pose){

    // find robot and do naming
    string name("Spherical");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);

    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    OdeRobot* sphere = 
      new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)), 
                               conf, name, 0.2); 
    sphere->place(pose);
    AbstractController* controller = new Sos();
    One2OneWiring*      wiring     = new One2OneWiring ( new WhiteUniformNoise() );
    OdeAgent*           agent      = new OdeAgent ( global );
    agent->init ( controller, sphere, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);      
    return agent;
  }

  OdeAgent* createSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                        GlobalData& global, osg::Matrix pose, string name = "Snake"){
    // find robot and do naming
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);        
    SchlangeConf snakeConf = SchlangeServo2::getDefaultConf();
    snakeConf.segmNumber   = 6+2*num;
    OdeRobot*    robot     = new SchlangeServo2 ( odeHandle, osgHandle, snakeConf, name);
    robot->place(pose); 

    AbstractController* controller = new Sox(1.0);     
    AbstractWiring*     wiring     = new One2OneWiring(new ColorUniformNoise(0.1));
    OdeAgent*           agent      = new OdeAgent( global );
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }

  OdeAgent* createHumanoid(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                           GlobalData& global, osg::Matrix pose){

    // find robot and do naming
    string name("Humanoid");
    int num = count_if(global.agents.begin(), global.agents.end(), agent_match_prefix(name));
    name += "_" + itos(num+1);
    
    return 0;
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      // lower case (create robot) but last creation less than a second ago
      if(key >= 'a' && key <='z' && lastRobotCreation > (global.time-1)) { 
        printf("You have to wait one second between robot creations.\n");
        return false;
      }

      switch ( (char) key )
	{
        case 'k': // test
          env.widthground=15;
          env.create(odeHandle, osgHandle, global,true);
        case 'b':
          createSpherical(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,2)); break;
        case 'B':
          removeRobot(global, "Spherical"); break;          
        case 'x':
          createHexapod(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,2)); break;
        case 'X':
          removeRobot(global, "Hexapod"); break;          
        case 's':
          createSnake(odeHandle, osgHandle, global, osg::Matrix::translate(4,4,2)); break;
        case 'S':
          removeRobot(global, "Snake"); break;          
        case 'i': // put Snake in center (useful in pit-mode
          createSnake(odeHandle, osgHandle, global, 
                      osg::Matrix::translate(0,0,2),"PitSnake"); break;
        case 'I':
          removeRobot(global, "PitSnake"); break;          
        case 'u':
          createHumanoid(odeHandle, osgHandle, global, osg::Matrix::translate(0,0,2)); break;
        case 'U':
          removeRobot(global, "Humanoid"); break;          
	default:
	  return false;
	  break;
	}
      if(key >= 'a' && key <='z') { // lower case -> created robot 
        lastRobotCreation = global.time;        
      }
      return true;
    }
    return false;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Simulation: b/B","add/remove Spherical");
    au.addKeyboardMouseBinding("Simulation: x/X","add/remove Hexapod");
    au.addKeyboardMouseBinding("Simulation: s/S","add/remove Snake");
    au.addKeyboardMouseBinding("Simulation: i/I","add/remove Snake in Pit");
    au.addKeyboardMouseBinding("Simulation: u/U","add/remove Humanoid");
    au.addKeyboardMouseBinding("Simulation: a/A","add/remove SliderArmband");
    au.addKeyboardMouseBinding("Simulation: d/D","add/remove Dog");
    au.addKeyboardMouseBinding("Simulation: l/L","add/remove LongVehicle");
  }


  
};



int main (int argc, char **argv)
{ 

  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
 
