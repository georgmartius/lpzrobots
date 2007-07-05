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
 *   Revision 1.4  2007-07-05 11:20:02  robot6
 *   hingeservo.h substituted by oneaxismotor.h (includes Hingeservo)
 *   "hand" added in Makefile
 *
 *   Revision 1.3  2007/05/07 09:07:21  robot3
 *   intended region for converting the code from nonreadable to human readable
 *
 *   Revision 1.2  2006/09/21 16:15:57  der
 *   *** empty log message ***
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

#include "simulation.h"

#include "odeagent.h"
#include "octaplayground.h"
#include "closedplayground.h"
#include "playground.h"
#include "passivesphere.h"
#include "passivebox.h"
#include "passivecapsule.h"
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include "hand.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;




AbstractController* controller;
motor teaching[3];

class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    //----for the first hand, the view from the right
    //    setCameraHomePos(Pos(8.28458, -0.182447, 3.5),  Pos(84.1294, -6.46484, 0));
    //----for both hands, the view from the top
    //setCameraHomePos(Pos(0.608728, -9.5055, 14.8199),  Pos(11.8571, -45.3423, 0));
    //----for the first hand the view from the top
    //setCameraHomePos(Pos(3.25291, -0.147082, 14.8199),  Pos(-92.7015, -98.0146, 0));
    //----to observe the ball on the palm of the hand.
    //setCameraHomePos(Pos(-4.56577, -0.127742, 5.61211),  Pos(-88.1353, -45.1827, 0));
    // the second hand 
    setCameraHomePos(Pos(-17.9977, 3.4133, 17.9206),  Pos(-88.7149, -43.5015, 0));

    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.05;
    global.odeConfig.setParam("gravity", -0.5);// dWorldSetGravity (world,0,0,-0.5);
    global.odeConfig.setParam("controlinterval", 5);


    // for(int i=0; i<50; i++){
    //      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
    //       s->setPosition(osg::Vec3(-4+(i/10),-4+(i%10),1)); 
    //       global.obstacles.push_back(s);    
    //     }
    
    OdeRobot *hand,*hand1;
    AbstractController *controller,*controller1;
    AbstractWiring *wiring,*wiring1;
    OdeAgent *agent,*agent1;

    HandConf conf = Hand::getDefaultConf();  
    conf.invert = 1; 
    conf.irRange = 0.7;
    conf.set_typ_of_motor = With_servo_motor;
    conf.show_contacts = true;
    conf.ir_sensor_used = true;
    conf.number_of_ir_sensors = conf.ir_sensor_used*15;
    conf.Draw_part_of_ir_sensor = Draw_All;
    conf.hand_is_drawn_under_angel = Is_Draw_under_180_degree;
    conf.jointLimit1 = -M_PI/180;
    conf.jointLimit2 = M_PI/2	;
    conf.servo_motor_Power = 1.2;
    conf.finger_winkel = M_PI/2;

    if(conf.hand_is_drawn_under_angel == Is_Draw_under_90_degree)
      {
	conf.x = -0.35;
	conf.y = 0;
	conf.z = 2;	   
      }
    else if (conf.hand_is_drawn_under_angel == Is_Draw_under_180_degree)
      {
	conf.x = -1.45;
	conf.y = 0.8;
	conf.z = 6.01;	   

      }
    //.changeColor(1,183,172)   
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    InvertMotorNStepConf cc5 = InvertMotorNStep::getDefaultConf();
    cc5.cInit=1.5;
    controller = new InvertMotorNStep(cc5); //SineController();//InvertMotorNStep(cc5);  
    //	  controller = new InvertMotorSpace(10);  
    agent = new OdeAgent(plotoptions);
    hand = new Hand(odeHandle, osgHandle,conf,"Hand");
    hand->setColor(Color(1.0,0.5,1.0));
    agent->init(controller, hand, wiring);
    controller->setParam("adaptrate", 0.000);
    //    controller->setParam("nomupdate", 0.0005);
    controller->setParam("epsC", 0.05);
    controller->setParam("epsA", 0.01);
    controller->setParam("epsC", 0.05);
    controller->setParam("rootE", 0);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 5);
    controller->setParam("s4del", 5);
    //	  controller->setParam("factorB",0);
	 
    hand->place(Pos(2.5,1.26,0));
    global.agents.push_back(agent);
    global.configs.push_back(controller); 
    global.configs.push_back(hand);

    	
     /* PassiveSphere* s = 
      new PassiveSphere(odeHandle, 
      osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
      s->setPosition(Pos(0,0.05,0.5+3)); 
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);         */
      

    //---For-the-reverse-hand

    //one hand directly above the over, so that the fingers can touch each other 
    conf.x = -3.45;
    conf.y = 4.8;
    conf.z = 2.9;	   
    conf.invert = -1; 
    conf.irRange = 0.7;
    conf.set_typ_of_motor = With_servo_motor;
    conf.show_contacts = true;
    conf.ir_sensor_used = true;
    conf.number_of_ir_sensors = conf.ir_sensor_used*15;
    conf.Draw_part_of_ir_sensor = Draw_All;
    conf.hand_is_drawn_under_angel = Is_Draw_under_180_degree;
    conf.jointLimit1 = -M_PI/180;
    conf.jointLimit2 = M_PI/2;
    conf.servo_motor_Power = 1.2;
    conf.finger_winkel = M_PI/2;

    wiring1 = new One2OneWiring(new ColorUniformNoise(0.1));
	
    controller1 = new InvertMotorNStep();  
    //	  controller = new InvertMotorSpace(10);  
    agent1 = new OdeAgent(plotoptions);
    hand1 = new Hand(odeHandle, osgHandle,conf,"Hand");
    hand1->setColor(Color(1.0,0.5,1.0));
    global.configs.push_back(controller1);
    agent1->init(controller1, hand1, wiring1);
    controller->setParam("adaptrate", 0.000);
    //    controller->setParam("nomupdate", 0.0005);
    controller->setParam("epsC", 0.05);
    controller->setParam("epsA", 0.01);
    controller->setParam("epsC", 0.05);
    controller->setParam("rootE", 0);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 5);
    controller->setParam("s4del", 5);
    //	  controller->setParam("factorB",0);
	 
    hand1->place(Pos(2.5,1.26,0));
    global.agents.push_back(agent1);


    ClosedPlayground* playground;
    if(conf.hand_is_drawn_under_angel == Is_Draw_under_180_degree){
      // use Playground as boundary:
      playground = new ClosedPlayground(odeHandle, osgHandle, osg::Vec3(4.7, 0.2, 26), 0.9);
      playground->setColor(Color(1.0f,0.0f,0.26f,0.0f));
      playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
      global.obstacles.push_back(playground);

    } 
    else if(conf.hand_is_drawn_under_angel == Is_Draw_under_90_degree)
      {
	// use Playground as boundary:
	//    playground = new ClosedPlayground(odeHandle, osgHandle, osg::Vec3(2.7, 0.2, 7), 1.2);

	//Version with closed playground 
	/*
	  playground = new ClosedPlayground(odeHandle, osgHandle, osg::Vec3(3.2, 0.2, 14), 1.0);

	  playground->setColor(Color(1.0f,0.0f,0.26f,0.1f));
	  playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
	*/
	Playground *playground1 = new Playground(odeHandle, osgHandle, osg::Vec3(4.7, 0.2, 10), 0.9);
	playground1->setColor(Color(1.0f,0.0f,0.26f,0.2f));
	playground1->setPosition(osg::Vec3(0,0,0)); // playground positionieren und g
	global.obstacles.push_back(playground1);
      }
   

    
      PassiveSphere* s = 
      new PassiveSphere(odeHandle, 
      osgHandle,1.3);
      s->setPosition(Pos(-0.4,0.45,0.5+7.4)); 
      s->setColor(Color(1.0f,0.2f,0.2f,0.5f));
      s->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(s); 
      showParams(global.configs);
    


    // the flashing box within a hand.
    /*
      PassiveBox* b = 
      new PassiveBox(odeHandle, 
      osgHandle, osg::Vec3(1.4,1.4,0.3),2);

      b->setPose(osg::Matrix::translate(conf.invert*(0+conf.x),(0.05+conf.y),(0.35+conf.z)));
      //setPosition(Pos(0,0.15,0.5+0.4)); 
      b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
      b->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(b); 
    */
    /*
      PassiveBox* b = 
      new PassiveBox(odeHandle, 
      osgHandle, osg::Vec3(1.8,2.8,1.8),5);
      b->setPosition(Pos(0,0.15,0.5+0.4)); 
      b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
      b->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(b); 

    */

    /*
      PassiveCapsule* c = 
      new PassiveCapsule(odeHandle, 
      osgHandle, 1,1,5);
      c->setPosition(Pos(0,0.05,0.5+2.4)); 
      c->setColor(Color(1.0f,0.2f,0.2f,0.5f));
      c->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(c); 
    */

    showParams(global.configs);
  }

  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;    
    bool handled = false;
    switch ( key )
      {
      case 's' :
	/*	controller->store("test") && printf("Controller stored\n");
		handled = true; 	*/
	break;
      case 'L' :
	/*	controller->restore("test") && printf("Controller loaded\n");
		handled = true; */
	break;	
	/*      
		case 'a': case'A':
		finger_force -= 0.3;
		handled = true; break;	
		case 'y': case 'Y':
		finger_force += 0.3;
		handled = true; break;	
		case ',':
		palm_torque += 0.3;
		handled = true; break;	
		case '.':
		palm_torque -= 0.3;
		handled = true; break;	
		case ' ':
		finger_force = 0;
		palm_torque = 0;
		thumb1=0;
		thumb2=0;
		thumb3=0;
		handled = true; break;	

		case '1':
		thumb1 -= 0.3;
		handled = true; break;	
		case '2':
		thumb1 += 0.3;
		handled = true; break;	
		case '3':
		thumb2 -= 0.3;
		handled = true; break;	
		case '4':
		thumb2 += 0.3;
		handled = true; break;	
		case '5':
		thumb3 -= 0.3;
		handled = true; break;	
		case '6':
		thumb3 += 0.3;
		handled = true; break;	

		case 'm':
		if ( (gripmode==lateral) && (dJointGetHingeAngle(joint[palm_index])<0.1) ){
		gripmode=precision;
		handled = true; break;	
		}
		if ( (gripmode==precision) && (dJointGetHingeAngle(joint[palm_index])<0.1) ){
		gripmode=lateral;
		handled = true; break;	
		}
	
		handled = true; break;	
	*/


	/*
	  case '10': {
	  FILE *f = fopen ("state.dif","wt");
	  if (f) {
	  dWorldExportDIF (world,f,"");
	  fclose (f);
	  }
	  }
	*/
      }
    /*
      std::cout<<"thumb_b pos: "<<dGeomGetPosition(beam[thumb_b].geom)[0]<<", "<<dGeomGetPosition(beam[thumb_b].geom)[1]<<", "<<dGeomGetPosition(beam[thumb_b].geom)[2]<<std::endl;
      std::cout<<"thumb_b rot: "<<dGeomGetRotation(beam[thumb_b].geom)[0]<<", "<<dGeomGetRotation(beam[thumb_b].geom)[1]<<", "<<dGeomGetRotation(beam[thumb_b].geom)[2]<<std::endl;
      std::cout<<"thumb_b ang: "<<
      dJointGetAMotorAngle (thumb_motor_joint, 0)<<", "<<
      dJointGetAMotorAngle (thumb_motor_joint, 1)<<", "<<
      dJointGetAMotorAngle (thumb_motor_joint, 2)<<std::endl;
      std::cout<<"thumb_t pos: "<<dGeomGetPosition(beam[thumb_t].geom)[0]<<", "<<dGeomGetPosition(beam[thumb_t].geom)[1]<<", "<<dGeomGetPosition(beam[thumb_t].geom)[2]<<std::endl;
      std::cout<<"thumb_t rot: "<<dGeomGetRotation(beam[thumb_t].geom)[0]<<", "<<dGeomGetRotation(beam[thumb_t].geom)[1]<<", "<<dGeomGetRotation(beam[thumb_t].geom)[2]<<std::endl;
      std::cout<<"thumb_t ang: "<<dJointGetHingeAngle(joint[thumb_bt])<<std::endl<<std::endl;

      std::cout<<"index_b ang: "<<dJointGetHingeAngle(joint[palm_index])<<std::endl<<std::endl;
    */

    fflush(stdout);
    return handled;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    /*
      au.addKeyboardMouseBinding("Finger_Force: a or A","decrement");
      au.addKeyboardMouseBinding("Finger_Force: y or Y","increment");
      au.addKeyboardMouseBinding("Palm_Torque: .","decrement");
      au.addKeyboardMouseBinding("Palm_Torque: ,","increment");
      au.addKeyboardMouseBinding("Change_AMotor_Value of thumb_motor_joint on 1-st 		axis : 1","decrement");
      au.addKeyboardMouseBinding("Change_AMotor_Value of thumb_motor_joint on 1-st 		axis : 2","increment");
      au.addKeyboardMouseBinding("Change_AMotor_Value of thumb_motor_joint on 2-d 		axis : 3","decrement");
      au.addKeyboardMouseBinding("Change_AMotor_Value of thumb_motor_joint on 2-d 		axis : 4","increment");
      au.addKeyboardMouseBinding("Change_AMotor_Value of thumb_motor_joint on 3-d 		axis : 5","decrement");
      au.addKeyboardMouseBinding("Change_AMotor_Value of thumb_motor_joint on 3-d 		axis : 6","increment");

      au.addKeyboardMouseBinding	("Finger_Force,Palm_Torque,AMotors_ofthumb_motor_joint:  ","Set to zero");
 
      au.addKeyboardMouseBinding("Gripmodes lateral or precision: m","toggle 		 	gripmode");
    */
    au.addKeyboardMouseBinding("Teachung: x","toggle mode");
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
 
