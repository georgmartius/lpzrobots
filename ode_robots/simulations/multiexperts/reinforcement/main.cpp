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
 *   Revision 1.1  2007-09-06 18:49:40  martius
 *   Sphere reinforcement
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
#include "playground.h"
#include "passivesphere.h"
#include "passivebox.h"
#include "complexplayground.h"

#include <selforg/ffnncontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/matrix.h>
#include <selforg/replaycontroller.h>
#include "multireinforce.h"

#include "sphererobot3masses.h"
#include "barrel2masses.h"
#include "axisorientationsensor.h"
#include "speedsensor.h"

#include "irsensor_wall.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
using namespace std;

// int sphereColl(dSurfaceParameters& params, GlobalData& globaldata, void *userdata, 
//                dContact* contacts, int numContacts,
//   dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2){
//   Substance::getSurfaceParams(params, s1, s2, globaldata.odeConfig.simStepSize);
//   params.slip1=params.slip2=0;
//   Substance::printSurfaceParams(params);
//   return 2;
// }



class ReinforceCmpStateController : public MultiReinforce {
public:
  ReinforceCmpStateController(const MultiReinforceConf& conf)
    : MultiReinforce(conf) {}

protected:
  virtual double calcReinforcement(){
    const Matrix& x_c = x_context_buffer[t%buffersize].map(fabs);
    const Matrix& speeds = x_c.rows(0,2);
    const Matrix& irs = x_c.rows(3,8);
//     double speed = max(speeds);
//     Matrix sspeeds(speeds);
//     sspeeds.toSort();
    
//    int rotaxis  = argmax(speeds);
    Matrix axisSensors(3,1);
    axisSensors.val(0,0)= irs.val(2,0) + irs.val(3,0);
    axisSensors.val(1,0)= irs.val(0,0) + irs.val(1,0);
    axisSensors.val(2,0)= irs.val(4,0) + irs.val(5,0);
    return 0.5*(speeds.val(1,0)-speeds.val(0,0)-speeds.val(2,0))-sqr(2*axisSensors.val(1,0));
    //    return 0.5*(speeds.val(1,0)-speeds.val(0,0)-speeds.val(2,0))-sqr(axisSensors.val(1,0)-0.3);
    //    return 0.5*(speed-sspeeds.val(0,0)-sspeeds.val(1,0))-sqr(2*axisSensors.val(rotaxis,0));
//     if(speed>0.3){
//       if(axisSensors.val(rotaxis,0)<0.25) return 1;
//       else return -1;
//     } else return 0;
      //    return -((x_c^T)*axisSensors).val(0,0);
  }

  virtual int getStateNumber(){
    return 8*sats.size();
  }

  virtual int calcState() {
    //  const Matrix& x_context = x_context_buffer[t%buffersize];
    const Matrix& x = x_buffer[t%buffersize];
    
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
    int agent=argmin(satErrors);
    
    /// IR state
    const Matrix& x_c = x_context_buffer[t%buffersize].map(fabs);
    const Matrix& irs = x_c.rows(3,8);
    int sensor=0;
    if(irs.val(0,0)>0.2) sensor |= 1; 
    if(irs.val(1,0)>0.2) sensor |= 2; 
    if(irs.val(2,0)+irs.val(3,0)+irs.val(4,0)+irs.val(5,0)>0.2) sensor |= 4; 

    list<pair<int,int> > sets;
    sets+= pair<int,int>(agent,sats.size());    
    sets+= pair<int,int>(sensor,8);
    return QLearning::valInCrossProd(sets);
  }
};

class ReinforceIRSphereController : public MultiReinforce {
public:
  ReinforceIRSphereController(const MultiReinforceConf& conf)
    : MultiReinforce(conf) { }

protected:
  virtual double calcReinforcement(){
    // the ir sensors for the rolling direction should be zero
    const Matrix& x = x_buffer[t%buffersize].map(fabs);
    const Matrix& x_c = x_context_buffer[t%buffersize].map(fabs);
    double speed = max(x_c);
    int rotaxis  = argmax(x_c);
    Matrix axisSensors(3,1);
    axisSensors.val(0,0)= x.val(2,0) + x.val(3,0);
    axisSensors.val(1,0)= x.val(0,0) + x.val(1,0);
    axisSensors.val(2,0)= x.val(4,0) + x.val(5,0);
    return 0.5-((speed>0.1) ? axisSensors.val(rotaxis,0) : 0);
      //    return -((x_c^T)*axisSensors).val(0,0);
  }

  virtual int getStateNumber(){
    return 3*2*3;
  }

  virtual int calcState() {
    const Matrix& x_c = x_context_buffer[t%buffersize];
    const Matrix& x = x_buffer[t%buffersize].map(fabs);
    
    int rotaxis  = argmax(x_c.map(fabs));
    int forward = x_c.val(rotaxis,0) > 0;

    int left=0, right=0;
    switch(rotaxis){
    case 0: left=2; right=3;
      break;
    case 1: left=0; right=1;
      break;
    case 2: left=4; right=5;
      break;
    default:
      break;
    }
    int sensors;
    if(x.val(left,0) + x.val(right,0)<0.2) sensors=1;
    else if(x.val(left,0)> x.val(right,0)) sensors=0;
    else  sensors=2;
    list<pair<int,int> > sets;
    sets+= pair<int,int>(sensors,3);    
    sets+= pair<int,int>(forward,2);
    sets+= pair<int,int>(rotaxis,3);    
    return QLearning::valInCrossProd(sets);
  }
};

class ReinforceSphereController : public MultiReinforce {
public:
  ReinforceSphereController(const MultiReinforceConf& conf)
    : MultiReinforce(conf) { }

protected:
  virtual double calcReinforcement(){
    // the ir sensors for the rolling direction should be zero
    const Matrix& x_c = x_context_buffer[t%buffersize].map(fabs);
    const Matrix& speeds = x_c.rows(0,2);
    const Matrix& irs = x_c.rows(3,8);
    double speed = max(speeds);
    int rotaxis  = argmax(speeds);
    Matrix axisSensors(3,1);
    axisSensors.val(0,0)= irs.val(2,0) + irs.val(3,0);
    axisSensors.val(1,0)= irs.val(0,0) + irs.val(1,0);
    axisSensors.val(2,0)= irs.val(4,0) + irs.val(5,0);
    if(speed>0.3){
      if(axisSensors.val(rotaxis,0)<0.25) return 1;
      else return -1;
    } else return 0;
  }

  virtual int getStateNumber(){
    return 3*2*3;
  }

  virtual int calcState() {
    const Matrix& x_c = x_context_buffer[t%buffersize];
    const Matrix& speeds = x_c.rows(0,2);
    const Matrix& irs = x_c.rows(3,8).map(fabs);
    
    int rotaxis  = argmax(speeds.map(fabs));
    int forward = speeds.val(rotaxis,0) > 0;

    int left=0, right=0;
    switch(rotaxis){
    case 0: left=2; right=3;
      break;
    case 1: left=0; right=1;
      break;
    case 2: left=4; right=5;
      break;
    default:
      break;
    }
    int sensors;
    if(irs.val(left,0) + irs.val(right,0)<0.2) sensors=1;
    else if(irs.val(left,0)> irs.val(right,0)) sensors=0;
    else  sensors=2;
    list<pair<int,int> > sets;
    sets+= pair<int,int>(sensors,3);    
    sets+= pair<int,int>(forward,2);
    sets+= pair<int,int>(rotaxis,3);    
    return QLearning::valInCrossProd(sets);
  }
};


class ThisSim : public Simulation {
public:
  AbstractController *controller;
  MultiReinforce* multirein;
  AbstractWiring* wiring;
  OdeAgent* agent;
  OdeRobot* sphere;
  Sphererobot3MassesConf conf;
  QLearning* qlearning;
  AbstractGround* playground;
  double playgroundsize[2];
  double playgroundheight;
  list<string> fs;

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
    global.time=200*60;

    //    global.odeConfig.setParam("realtimefactor",0);
//     global.odeConfig.setParam("drawinterval",2000); 

    bool labyrint=false;      
    bool roundcorridor=false;
    bool longsquarecorridor=false; 
    bool complexpl=true; 

    bool useIRSats=false;
    playgroundheight=10;

    playground=0;    
    if(longsquarecorridor){
      playgroundsize[0] = 20;
      playgroundsize[1] = 100;
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(playgroundsize[0], 0.2, playgroundheight ), 
				  playgroundsize[1]/playgroundsize[0], false);
      playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
      playground->setGroundTexture("Images/dusty.rgb");    
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.6));
      playground->setPosition(osg::Vec3(0,0,-playgroundheight/2));
      playground->setTexture("");
      global.obstacles.push_back(playground);
      // underground
//       playground = new Playground(odeHandle, osgHandle,osg::Vec3(playgroundsize[0], 0, 0 ), 
// 				  playgroundsize[1]/playgroundsize[0], true);
//       playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
//       playground->setGroundTexture("Images/dusty.rgb");    
//       playground->setPosition(osg::Vec3(0,0,0.1));
//       global.obstacles.push_back(playground);
      
    }
    if(roundcorridor){
      AbstractGround* playground = new OctaPlayground(odeHandle, osgHandle,osg::Vec3(15, 0.2, playgroundheight ), 
						      12, true);
      playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
      playground->setGroundTexture("Images/dusty.rgb");    
      playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.1));
      playground->setPosition(osg::Vec3(0,0,0.1));
      playground->setTexture("");
      global.obstacles.push_back(playground);
      //     // inner playground
      playground = new OctaPlayground(odeHandle, osgHandle,osg::Vec3(10, 0.2, playgroundheight), 12, false);
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
    if(complexpl){
      double factor=5;
      playground = new ComplexPlayground(odeHandle, osgHandle, "labyrint42.fig", factor, 0.3,false);
      //playground = new ComplexPlayground(odeHandle, osgHandle, "labyrint.fig", factor, 0.05);
      playground->setPosition(osg::Vec3(0,0,-2));
      global.obstacles.push_back(playground);
    }

    
    sphere=0;
  
    //****************
    conf = Sphererobot3Masses::getDefaultConf();  
    conf.pendularrange  = 0.3; 
    conf.brake          = 0.1;
//     conf.pendularrange  = 0.25; 
//     conf.motorpowerfactor  = 150;    
    conf.motorsensor=false;
//     //    conf.spheremass  = 1;    
//     conf.spheremass  = 0.3;  
    if(useIRSats){
      conf.irAxis1=true;
      conf.irAxis2=true;
      conf.irAxis3=true;
      conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));
      conf.motor_ir_before_sensors=true;
    }else{
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));
      conf.irAxis1=true;
      conf.irAxis2=true;
      conf.irAxis3=true;
      conf.motor_ir_before_sensors=false;    
    }
    list<dGeomID> avoids;
    avoids.push_back(ground);
    conf.irSensorTempl=new IRSensorWall(0.8,avoids);
    conf.irsensorscale=5;
    
    // create new sphere
    OdeHandle sphereOdeHandle = odeHandle;
    sphereOdeHandle.substance.roughness=3;
    sphereOdeHandle.substance.hardness =100;
    //     sphereOdeHandle.substance.setCollisionCallback(sphereColl, 0);
    sphere = new Sphererobot3Masses ( sphereOdeHandle, osgHandle.changeColor(Color(0,0.0,2.0)), 
				      conf, "Sphere_Reinf_square", 0.3); 
    
    if(roundcorridor){
      sphere->place(Pos(11,0,0.1));
      setCameraHomePos(Pos(8.73231, 16.2483, 4.53275),  Pos(-171.649, -14.6511, 0));
    }
    else{
      sphere->place(Pos(0,0,0.1));
    }

    
    MultiReinforceConf msc = MultiReinforce::getDefaultConf();
    msc.tauE1=25;
    msc.tauH=20;
    msc.tauI=50;
    if(useIRSats){
      msc.numContext = 3;
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_00_00.net");
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_02_00.net");
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_03_00.net");
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_06_00.net");
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_08_00.net");
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_10_00.net");
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_13_00.net");
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_16_00.net");
      fs+=string("../create_sphere_ir_sats/sats/IR_Sphere_18_00.net");
    } else {
      msc.numContext = 9;
//       fs+=string("../create_sphere_ir_sats/sats/Multi20_2h_Sphere_nogat_180min_02.net");
//       fs+=string("../create_sphere_ir_sats/sats/Multi20_2h_Sphere_nogat_180min_03.net");
//       fs+=string("../create_sphere_ir_sats/sats/Multi20_2h_Sphere_nogat_180min_00.net");
//       fs+=string("../create_sphere_ir_sats/sats/Multi20_2h_Sphere_nogat_180min_06.net");
//       fs+=string("../create_sphere_ir_sats/sats/Multi20_2h_Sphere_nogat_180min_13.net");
//       fs+=string("../create_sphere_ir_sats/sats/Multi20_2h_Sphere_nogat_180min_08.net");
      msc.useY=false;
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_00.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_01.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_02.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_03.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_04.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_05.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_06.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_07.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_08.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_09.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_10.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_11.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_12.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_13.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_14.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_15.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_16.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_17.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_18.net");
//       fs+=string("../MinMod/test1/Multi20_3_noY_test_19.net");
//      fs+=string("../MinMod/Sphere_Green/Sphere_green_left.net");
      fs+=string("../MinMod/Sphere_Green/Sphere_green_fw.net.ascii");
      //      fs+=string("../MinMod/Sphere_Green/Sphere_green_fw_r1.net.ascii");
      fs+=string("../MinMod/Sphere_Green/Sphere_green_fw_r2.net.ascii");
      //      fs+=string("../MinMod/Sphere_Green/Sphere_green_fw_l1.net.ascii");
      fs+=string("../MinMod/Sphere_Green/Sphere_green_fw_l2.net.ascii");
      fs+=string("../MinMod/Sphere_Green/Sphere_green_bw.net.ascii");
      //      fs+=string("../MinMod/Sphere_Green/Sphere_green_bw_l1.net.ascii");
      fs+=string("../MinMod/Sphere_Green/Sphere_green_bw_r2.net.ascii");
      //      fs+=string("../MinMod/Sphere_Green/Sphere_green_bw_l1.net.ascii");
      fs+=string("../MinMod/Sphere_Green/Sphere_green_bw_l2.net.ascii");
    }
    msc.satFiles      = fs;
    
    qlearning = new QLearning(0.1, 0.90, 0.1, 3,true,true);
    msc.qlearning=qlearning;
    msc.useDerive=false;
    
    if(useIRSats)
      multirein = new ReinforceIRSphereController(msc);
    else
      multirein = new ReinforceCmpStateController(msc);

    multirein->setParam("mancontrol",0);
    multirein->setParam("action",0);
    
    wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
    agent = new OdeAgent ( plotoptions );
    agent->init ( multirein , sphere , wiring );
    //    agent->setTrackOptions(TrackRobot(true,true,false,true,"",50));
    global.agents.push_back ( agent );
    
    global.configs.push_back(multirein);
    global.configs.push_back(msc.qlearning);
    showParams(global.configs);
  }


  virtual void addCallback(GlobalData& global, bool draw, bool pause, bool control) {
    if(sphere){
      dBodyID b = sphere->getMainPrimitive()->getBody();
      double brake=-0.25;
      const double* vel = dBodyGetAngularVel( b);
      dBodyAddTorque ( b , brake*vel[0] , brake*vel[1] , brake*vel[2] );    
    }

  }
  
  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    char file[256];    FILE* f;
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	case 'y' : dBodyAddForce ( sphere->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
	case 'Y' : dBodyAddForce ( sphere->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
	case 'x' : dBodyAddTorque ( sphere->getMainPrimitive()->getBody() , 0 , 0 , 10 ); break;
	case 'X' : dBodyAddTorque ( sphere->getMainPrimitive()->getBody() , 0 , 0 , -10 ); break;
	case 'k' : 
	  if(playground){
	    playgroundsize[0]=std::max(2.0,playgroundsize[0] - 0.5);
	    playground->changeGeometry(playgroundsize[0],0.2,playgroundheight,playgroundsize[1]/playgroundsize[0]);
	    cout << "Change playground to : " << playgroundsize[0] << endl;
	  }
	  break;
	case 'i' : 
	  if(playground){
	    playgroundsize[0]+=0.5;
	    playground->changeGeometry(playgroundsize[0],0.2,playgroundheight,playgroundsize[1]/playgroundsize[0]);
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
	case 'd' : 
	  multirein->storeSats(fs);  
	  cout << "saved Sats!" << endl; 
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
  sim.setCaption("robot.informatik.uni-leipzig.de    Martius,Der,Herrmann 2007");
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;  
}
 
