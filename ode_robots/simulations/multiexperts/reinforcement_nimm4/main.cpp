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
 *   Revision 1.1  2007-08-29 15:32:52  martius
 *   reinforcement learning with 4 wheeled
 *
 *   Revision 1.3  2007/08/06 14:25:57  martius
 *   new version without gating network
 *
 *   Revision 1.2  2007/06/18 08:11:22  martius
 *   nice version with many agents
 *
 *   Revision 1.1  2007/06/14 07:59:30  martius
 *   nice scanning of initial parameters
 *
 *
 ***************************************************************************/

#include "simulation.h"

#include "odeagent.h"
#include "octaplayground.h"
#include "complexplayground.h"
#include "playground.h"
#include "passivesphere.h"
#include "passivebox.h"

#include <selforg/ffnncontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/matrix.h>
#include <selforg/replaycontroller.h>
#include "multireinforce.h"

#include "fourwheeled.h"
#include "addsensors2robotadapter.h"
#include "axisorientationsensor.h"
#include "speedsensor.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
using namespace std;


class ReinforceCmpStateIRController : public MultiReinforce {
public:
  ReinforceCmpStateIRController(const MultiReinforceConf& conf)
    : MultiReinforce(conf) {}

protected:
  virtual double calcReinforcement(){
    const Matrix& x_c = x_context_buffer[t%buffersize].map(fabs);
    // 0 to 5 are IR
    int largest = argmax(x_c.rows(0,5));
    // 6 is speed    
    return 0.5*x_c.val(6,0)-sqr(2*x_c.val(largest,0));
  }

  virtual int getStateNumber(){
    return sats.size() * 7;
  }

  virtual int calcState() {
    //  const Matrix& x_context = x_context_buffer[t%buffersize];
    const Matrix& x = x_buffer[t%buffersize];
    const Matrix& x_c = x_context_buffer[t%buffersize];
    const Matrix& irs = x_c.rows(0,5).map(fabs); // 0 to 5 are IR 
    const Matrix& x_tm1 = x_buffer[(t-1)%buffersize];
    const Matrix& x_tm2 = x_buffer[(t-2)%buffersize];
    const Matrix& xp_tm1 = xp_buffer[(t-1)%buffersize];
    //  const Matrix& y_tm1 = y_buffer[(t-1)%buffersize];
    const Matrix& y_tm2 = y_buffer[(t-2)%buffersize];
    
    // depending on useDerive we have
    // we have to use F(x_{t-1},x_{t-2} | \dot x_{t-1} ,y_{t-2}) -> (x_t, y_{t-1}) for the sat network
    
    if(conf.useDerive){
      satInput   = x_tm1.above(xp_tm1);
    } else {
      satInput   = x_tm1.above(x_tm2);
    }
    if(conf.useY){
      satInput.toAbove(y_tm2);
    }
    
    // ask all networks to make there predictions on last timestep, compare with real world
    assert(satErrors.getM()>=sats.size());
    
    unsigned int i=0;
    int x_s = x.getM();
    FOREACH(vector<Sat>, sats, s){
      const Matrix& out = s->net->process(satInput);
      satErrors.val(i,0) =  (x-out.rows(0,x_s-1)).multTM().val(0,0);
      i++;
    }

    // IR sensors    
    // group them into none, front, left, back, right
    Matrix irgroups(5,1);
    irgroups.val(0,0)=0.2; // threshhold;
    irgroups.val(1,0)=max(irs.val(0,0),irs.val(1,0));
    irgroups.val(2,0)=irs.val(2,0);
    irgroups.val(3,0)=max(irs.val(3,0),irs.val(4,0));
    irgroups.val(4,0)=irs.val(5,0);
    
    int sensor = argmax(irgroups);
    Matrix irgroups2(irgroups);
    irgroups2.val(sensor,0)=0;
    if(max(irgroups2)>0.4) sensor= (sensor<3) ? 5 : 6;

    list<pair<int,int> > sets;
    sets+= pair<int,int>(sensor,7);    
    sets+= pair<int,int>(argmin(satErrors),sats.size());
    return QLearning::valInCrossProd(sets);
  }
};


class ReinforceNimm4Controller : public MultiReinforce {
public:
  ReinforceNimm4Controller(const MultiReinforceConf& conf)
    : MultiReinforce(conf) { }

protected:
  virtual double calcReinforcement(){
    const Matrix& x_c = x_context_buffer[t%buffersize].map(fabs);
    // 0 to 5 are IR
    int largest = argmax(x_c.rows(0,5));
    // 6 is speed    
    return 0.5*x_c.val(6,0)-x_c.val(largest,0);
  }

  virtual int getStateNumber(){
    return 7*2;
  }

  virtual int calcState() {
    const Matrix& x_c = x_context_buffer[t%buffersize];
    const Matrix& irs = x_c.rows(0,5).map(fabs);    
    // 0 to 5 are IR
    int largest = argmax(irs);
    int sensor=0;
    if(irs.val(largest,0) > 0.2){
      sensor=largest+1;
    }
    int direction = x_c.val(6,0) > 0;
    list<pair<int,int> > sets;
    sets+= pair<int,int>(sensor,7);    
    sets+= pair<int,int>(direction,2);
    return QLearning::valInCrossProd(sets);
  }
};


class ThisSim : public Simulation {
public:
  AbstractController *controller;
  MultiReinforce* multirein;
  AbstractWiring* wiring;
  OdeAgent* agent;
  OdeRobot* robot;
  QLearning* qlearning;
  AbstractGround* playground;
  double playgroundsize[2];

  ThisSim(){}

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    // initialization
    global.odeConfig.setParam("noise",0.05); 
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("realtimefactor",1);

    //    global.odeConfig.setParam("realtimefactor",0);
//     global.odeConfig.setParam("drawinterval",2000); 

    bool labyrint=false;      
    bool roundcorridor=false;
    bool longsquarecorridor=false; 
    bool complexpl=true; 


    playground=0;    
    if(longsquarecorridor){
      playgroundsize[0] = 50;
      playgroundsize[1] = 500;
      playgroundsize[0] = 10;
      playgroundsize[1] = 30;
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(playgroundsize[0], 0.2, 1.2 ), 
				  playgroundsize[1]/playgroundsize[0], false);
      playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
      playground->setGroundTexture("Images/dusty.rgb");    
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.6));
      playground->setPosition(osg::Vec3(0,0,0.1));
      playground->setTexture("");
      global.obstacles.push_back(playground);
    }
    if(roundcorridor){
      playground = new OctaPlayground(odeHandle, osgHandle,osg::Vec3(15, 0.2, 1.2 ), 12, true);
      playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
      playground->setGroundTexture("Images/dusty.rgb");    
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.1));
      playground->setPosition(osg::Vec3(0,0,0.1));
      playground->setTexture("");
      global.obstacles.push_back(playground);
      //     // inner playground
      playground = new OctaPlayground(odeHandle, osgHandle,osg::Vec3(10, 0.2, 1.2), 12, false);
      playground->setColor(Color(255/255.0,200/255.0,0/255.0, 0.1));
      playground->setTexture("");
      playground->setPosition(osg::Vec3(0,0,0.1));
      global.obstacles.push_back(playground);
    }

    if(labyrint){
      double radius=10.5;
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(radius*2+1, 0.2, 5 ), 1);
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
    if(complexpl){
      double factor=2;
      playground = new ComplexPlayground(odeHandle, osgHandle, "labyrint42.fig", factor, 0.05);
      //playground = new ComplexPlayground(odeHandle, osgHandle, "labyrint.fig", factor, 0.05);
      playground->setPosition(osg::Vec3(0,0,0.1));
      global.obstacles.push_back(playground);
    }
    

    
  
    //****************
    
    // create new robot
    //    OdeHandle sphereOdeHandle = odeHandle;
    //     sphereOdeHandle.substance.setCollisionCallback(sphereColl, 0);
    FourWheeledConf fwc = FourWheeled::getDefaultConf();
    fwc.irRange=1.5;
    fwc.irFront=true;
    fwc.irBack=true;
    fwc.irSide=true;
    OdeRobot* nimm4 = new FourWheeled ( odeHandle, osgHandle, fwc, "4Wheel_Reinf56_"); 
    AddSensors2RobotAdapter* addsensorrobot =  
      new AddSensors2RobotAdapter(odeHandle, osgHandle, nimm4);
    addsensorrobot->addSensor(new SpeedSensor(1,SpeedSensor::TranslationalRel, Sensor::Z));
    addsensorrobot->addSensor(new SpeedSensor(1,SpeedSensor::RotationalRel, Sensor::X));
    robot = addsensorrobot;
        
    if(roundcorridor){
      robot->place(Pos(11,0,0.1));
      setCameraHomePos(Pos(8.73231, 16.2483, 4.53275),  Pos(-171.649, -14.6511, 0));
    }
    else{
      robot->place(Pos(0,0,0.1));
    }

    MultiReinforceConf msc = MultiReinforce::getDefaultConf();
    msc.tauE1=5;
    msc.tauH=3;
    msc.tauI=10;
    list<string> fs;
    {
      msc.numContext = 8;
      msc.useY=false;
      fs+=string("4Wheel_30min_00.net");
      fs+=string("4Wheel_30min_01.net");
      fs+=string("4Wheel_30min_02.net");
      fs+=string("4Wheel_30min_03.net");
      fs+=string("4Wheel_30min_04.net");
      fs+=string("4Wheel_30min_05.net");
      fs+=string("4Wheel_30min_06.net");
      fs+=string("4Wheel_30min_07.net");
    }
    msc.satFiles      = fs;
    
    qlearning = new QLearning(0.05, 0.90, 0.1, 3,true,true);
    msc.qlearning=qlearning;
    msc.useDerive=false;
  
    //    multirein = new ReinforceNimm4Controller(msc);
    multirein = new ReinforceCmpStateIRController(msc);
        
    
    wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
    agent = new OdeAgent ( plotoptions );
    agent->init ( multirein , robot , wiring );
    agent->setTrackOptions(TrackRobot(true,true,false,true,"Labyrint",20));
    global.agents.push_back ( agent );
    
    global.configs.push_back(multirein);
    global.configs.push_back(msc.qlearning);
    showParams(global.configs);
  }


  virtual void addCallback(GlobalData& global, bool draw, bool pause, bool control) {
  }
  
  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    char file[256];
    FILE* f;
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	case 'y' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
	case 'Y' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
	case 'x' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , 0 , 10 ); break;
	case 'X' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , 0 , -10 ); break;
	case 'k' : 
	  if(playground){
	    playgroundsize[0]=std::max(2.0,playgroundsize[0] - 0.5);
	    playground->changeGeometry(playgroundsize[0],0.2,1.2,playgroundsize[1]/playgroundsize[0]);
	    cout << "Change playground to : " << playgroundsize[0] << endl;
	  }
	  break;
	case 'i' : 
	  if(playground){
	    playgroundsize[0]+=0.5;
	    playground->changeGeometry(playgroundsize[0],0.2,1.2,playgroundsize[1]/playgroundsize[0]);
	    cout << "Change playground to : " << playgroundsize[0] << endl;
	  }
	  break;
	case 's' : 
	  sprintf(file,"QTable%07.1f.matrix",globalData.time);
	  f= fopen(file,"w");
	  if(!f) cerr << "cannot open file: " << file << endl;
	  else {
	    qlearning->getQ().write(f); 
	    fclose(f);
	    cout << "saved Qtable to "<<  file << endl; 
	  }
	  return true;
	  break;
	case 'c' : 
	  sprintf(file,"contour.dat");
	  f= fopen(file,"w");
	  if(!f) cerr << "cannot open file: " << file << endl;
	  else {
	    if(playground) playground->printContours(f);
	    fclose(f);
	    cout << "saved Contour to "<<  file << endl; 
	  }
	  return true;
	  break;
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
 
