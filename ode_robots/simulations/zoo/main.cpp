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

// template <class T>
// bool removeFromList(T t, )
//       ObstacleList::iterator i = find(global.obstacles.begin(), global.obstacles.end(), 
//                                       playground);
//       if(i != global.obstacles.end()){
//         global.obstacles.erase(i);
//       }


class ThisSim : public Simulation {
public:

  Env env;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    int numCater=1;
    int numSchlangeL=4;
    int numNimm2=2;
    int numNimm4=0;
    int numHurling=1;
    int numSphere=1;
    int numSliderWheele=0;
    bool hexapod=true;

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

    for(int i=0; i<5; i++){
      PassiveSphere* s = 
	new PassiveSphere(odeHandle, 
			  osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
      s->setPosition(Pos(i*0.5-2, i*0.5, 1.0)); 
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);    
    }

    for(int i=0; i<5; i++){
      PassiveBox* b = 
	new PassiveBox(odeHandle, 
			  osgHandle, osg::Vec3(0.2+i*0.1,0.2+i*0.1,0.2+i*0.1));
      b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
      b->setTexture("Images/light_chess.rgb");
      b->setPosition(Pos(i*0.5-5, i*0.5, 1.0)); 
      global.obstacles.push_back(b);    
    }

    for(int i=0; i<5; i++){
      PassiveCapsule* c = 
	new PassiveCapsule(odeHandle, osgHandle, 0.2f, 0.3f, 0.3f);
      c->setPosition(Pos(i-1, -i, 1.0)); 
      c->setColor(Color(0.2f,0.2f,1.0f,0.5f));
      c->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(c);    
    }
        
    OdeAgent* agent;
    AbstractWiring* wiring;
    OdeRobot* robot;
    AbstractController *controller;
    
    if(hexapod){
      HexapodConf myHexapodConf = Hexapod::getDefaultConf();
      myHexapodConf.coxaPower= .8;
      myHexapodConf.tebiaPower= .5;
      myHexapodConf.coxaJointLimitV =.9;// M_PI/8;  ///< angle range for vertical direction of legs
      myHexapodConf.coxaJointLimitH = 1.3;//M_PI/4;
      myHexapodConf.tebiaJointLimit = 1.8;// M_PI/4; // +- 45 degree
      myHexapodConf.percentageBodyMass=.5;
      myHexapodConf.useBigBox=true;
      myHexapodConf.tarsus=true;
      myHexapodConf.numTarsusSections = 1;
      myHexapodConf.useTarsusJoints = true;
      
      OdeHandle rodeHandle = odeHandle;
      rodeHandle.substance.toRubber(20);    
      robot = new Hexapod(rodeHandle, osgHandle.changeColor(Color(1,1,1)), 
			  myHexapodConf, "Hexapod");
      robot->place(osg::Matrix::rotate(M_PI*0,1,0,0)*osg::Matrix::translate(0,0,1+ 2));

      controller = new Sox(1.2, false);
      controller->setParam("epsC",0.3);
      controller->setParam("epsA",0.05);
      controller->setParam("Logarithmic",1);
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( global);
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(robot);   
    }

    

    //******* R A U P E  *********/
    for(int r=0; r < numCater ; r++) { 
      CaterPillar* myCaterPillar;
      CaterPillarConf myCaterPillarConf = DefaultCaterPillar::getDefaultConf();
      myCaterPillarConf.segmNumber=3+r;
      myCaterPillarConf.jointLimit=M_PI/3;
      myCaterPillarConf.motorPower=0.2;
      myCaterPillarConf.frictionGround=0.01;
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
    
    
    //******* S C H L A N G E  (Long)  *********/
    for(int r=0; r < numSchlangeL ; r++) { 
      SchlangeServo2* snake;
      SchlangeConf snakeConf = SchlangeServo2::getDefaultConf();
      snakeConf.segmNumber=6+r;
      snakeConf.frictionGround=0.01;
      snake = new SchlangeServo2 ( odeHandle, osgHandle, snakeConf, "SchlangeLong" + std::itos(r));
      ((OdeRobot*) snake)->place(Pos(4,4-r,r)); 
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit=2.0;
      controller = new InvertMotorNStep(invertnconf);     
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( global, PlotOption(NoPlot) );
      agent->init(controller, snake, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(snake);   
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

    //****** S P H E R E **********/
    for(int r=0; r < numSphere; r++) {
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      Sphererobot3Masses* sphere1 = 
	new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)), 
				 conf, "Sphere" + std::itos(r), 0.2); 
      ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.1 ));
      controller = new InvertMotorSpace(15);
      controller->setParam("sinerate", 40);  
      controller->setParam("phaseshift", 0.0);
      One2OneWiring* wiring2 = new One2OneWiring ( new ColorUniformNoise() );
      agent = new OdeAgent ( global );
      agent->init ( controller , sphere1 , wiring2 );
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
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
  }
  
  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
        case 'k':
          env.widthground=5;
          env.create(odeHandle, osgHandle, global,true);
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
 
