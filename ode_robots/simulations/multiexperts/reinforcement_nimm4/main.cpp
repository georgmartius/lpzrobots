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
 *   Revision 1.7  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.6  2009/01/20 17:29:52  martius
 *   cvs commit
 *
 *   Revision 1.5  2008/11/14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.4  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.3  2008/04/22 15:22:55  martius
 *   removed test lib and inc paths from makefiles
 *
 *   Revision 1.2  2007/09/06 18:49:56  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2007/08/29 15:32:52  martius
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

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>

#include <selforg/ffnncontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/matrix.h>
#include <selforg/replaycontroller.h>
#include "multireinforce.h"
#include <selforg/classicreinforce.h>

#include <ode_robots/fourwheeled.h>
#include <ode_robots/addsensors2robotadapter.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
using namespace std;

enum PLAYGROUND {none, labyrint, complexpl, roundcorridor, longsquarecorridor, cluttered} playgr;

bool track=false;
int interval=10;
bool useMultiAgent=true;
string absolutePath="";
double realtimefactor=1;
bool rareprint=false;
int rewavg = 5;

class ReinforceCmpStateIRController : public MultiReinforce {
public:
  ReinforceCmpStateIRController(const MultiReinforceConf& conf)
    : MultiReinforce(conf) {}

protected:
  virtual double calcReinforcemen2(){
    const Matrix& x_c = x_context_buffer[t%buffersize].map(fabs);
    // 0 to 5 are IR
    int largest = argmax(x_c.rows(0,5));
    // 6 is speed
    return 0.5*x_c.val(6,0)-sqr(2*x_c.val(largest,0));

  }

  virtual double calcReinforcement1(){
    const Matrix& x_c = x_context_buffer[t%buffersize].map(fabs);
    int largest = argmax(x_c.rows(0,5));
    if(x_c.val(largest,0)>0.4){
      return -1;
    }else{
      return x_c.val(6,0);
    }
  }

  virtual double calcReinforcement(){
    const Matrix& x_c = x_context_buffer[t%buffersize];
    int largest = argmax(x_c.rows(0,5));
    if(x_c.val(largest,0)>0.4){
      return -1;
    }else{
      return -x_c.val(6,0);
    }
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
    //    sets+= pair<int,int>(argmin(satErrors),sats.size());
    sets+= pair<int,int>(action,sats.size());
    return QLearning::valInCrossProd(sets);
  }
};


class ReinforceIRController : public MultiReinforce {
public:
  ReinforceIRController(const MultiReinforceConf& conf)
    : MultiReinforce(conf) {}

protected:
  virtual double calcReinforcement(){
    const Matrix& x_c = x_context_buffer[t%buffersize];
    int largest = argmax(x_c.rows(0,5));
    if(x_c.val(largest,0)>0.6){
      return -1;
    } else if(x_c.val(largest,0)>0.2){
      return clip(-x_c.val(6,0),-0.2,2.0); // only consider speed
    }else{
      // driving (backwards, because forward expert makes curve) and few turning
      return -x_c.val(6,0)-0.5*fabs(x_c.val(7,0));
    }
  }

  virtual int getStateNumber(){
    return 2*2*2 * 3;
  }

  virtual int calcState() {
    const Matrix& x_c = x_context_buffer[t%buffersize];
    // IR sensors
    const Matrix& irs = x_c.rows(0,5).map(fabs); // 0 to 5 are IR

    Matrix irgroups(3,1);
    // side sensors
    irgroups.val(0,0)=0.11; // threshhold
    irgroups.val(1,0)=irs.val(2,0);
    irgroups.val(2,0)=irs.val(5,0);
    int sensorside = argmax(irgroups);

    //    int direction = x_c.val(6,0) > 0;

    list<pair<int,int> > sets;
    sets+= pair<int,int>(irs.val(0,0)>0.11,2);// front left
    sets+= pair<int,int>(irs.val(1,0)>0.11,2);// front right
    sets+= pair<int,int>(max(irs.val(3,0),irs.val(4,0)) >0.11, 2);// rear
    sets+= pair<int,int>(sensorside,3);
    //    sets+= pair<int,int>(direction,2);
    //    sets+= pair<int,int>(action,sats.size());
    return QLearning::valInCrossProd(sets);
  }
};


class ReinforceNimm4Controller : public ClassicReinforce {
public:
  ReinforceNimm4Controller(const ClassicReinforceConf& conf)
    : ClassicReinforce(conf) {  }

protected:
  virtual double calcReinforcement(){
    const Matrix& x_c = x_context_buffer[t%buffersize];
    int largest = argmax(x_c.rows(0,5));
    if(x_c.val(largest,0)>0.6){
      return -1;
    } else if(x_c.val(largest,0)>0.2){
      return clip(-x_c.val(6,0),-0.2,2.0); // only consider speed
    }else{
      // driving (backwards, because forward expert makes curve) and few turning
      return -x_c.val(6,0)-0.5*fabs(x_c.val(7,0));
    }
  }

  virtual int getStateNumber(){
    return 2*2*2 * 3;
  }

  virtual int calcState() {
    const Matrix& x_c = x_context_buffer[t%buffersize];
    // IR sensors
    const Matrix& irs = x_c.rows(0,5).map(fabs); // 0 to 5 are IR

    Matrix irgroups(3,1);
    // side sensors
    irgroups.val(0,0)=0.11; // threshhold
    irgroups.val(1,0)=irs.val(2,0);
    irgroups.val(2,0)=irs.val(5,0);
    int sensorside = argmax(irgroups);

    //    int direction = x_c.val(6,0) > 0;

    list<pair<int,int> > sets;
    sets+= pair<int,int>(irs.val(0,0)>0.11,2);// front left
    sets+= pair<int,int>(irs.val(1,0)>0.11,2);// front right
    sets+= pair<int,int>(max(irs.val(3,0),irs.val(4,0)) >0.11, 2);// rear
    sets+= pair<int,int>(sensorside,3);
    //    sets+= pair<int,int>(direction,2);
    //    sets+= pair<int,int>(action,sats.size());
    return QLearning::valInCrossProd(sets);
  }

  virtual int getActionNumber(){
    return 3*3*3*3;
  }

  virtual matrix::Matrix calcMotor(int action){
    list<int> ranges;
    ranges += 3;
    ranges += 3;
    ranges += 3;
    ranges += 3;
    list<int> vals = QLearning::ConfInCrossProd(ranges, action);
    Matrix a(4,1);
    int i=0;
    FOREACHC(list<int>,vals,v){
      a.val(i,0)=(*v-1)*0.57;
      i++;
    }
    return a;
  }

};


class ThisSim : public Simulation {
public:
  AbstractController *controller;
  //mult  MultiReinforce* multirein;
  AbstractWiring* wiring;
  OdeAgent* agent;
  OdeRobot* robot;
  QLearning* qlearning;
  AbstractGround* playground;
  double playgroundsize[2];
  FILE* log;


  ThisSim(QLearning* qlearning):
    qlearning(qlearning), log(0) { }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    // initialization
    global.odeConfig.setParam("noise",0.05);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",2);
    global.odeConfig.setParam("realtimefactor",realtimefactor);

    //    global.odeConfig.setParam("realtimefactor",0);
//     global.odeConfig.setParam("drawinterval",2000);

    playground=0;
    switch(playgr){
    case longsquarecorridor:
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
      break;
    case roundcorridor:
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
      break;
    case cluttered:
      {
        double radius=12.5;
        playground = new Playground(odeHandle, osgHandle,osg::Vec3(radius*2+1, 0.2, 5 ), 1);
        playground->setGroundColor(Color(255/255.0,200/255.0,0/255.0));
        playground->setGroundTexture("Images/really_white.rgb");
        playground->setColor(Color(255/255.0,200/255.0,21/255.0, 0.1));
        playground->setPosition(osg::Vec3(0,0,0.1));
        playground->setTexture("");
        global.obstacles.push_back(playground);
        //      int obstanz=30;
        int obstanz=10;
        OsgHandle rotOsgHandle = osgHandle.changeColor(Color(255/255.0, 47/255.0,0/255.0));
        OsgHandle gruenOsgHandle = osgHandle.changeColor(Color(0,1,0));
        for(int i=0; i<obstanz; i++){
          PassiveBox* s = new PassiveBox(odeHandle, (i%2)==0 ? rotOsgHandle : gruenOsgHandle,
                                         osg::Vec3(random_minusone_to_one(0)+1.8,
                                                   random_minusone_to_one(0)+1.8 ,1),15);
          s->setPose(osg::Matrix::translate(radius/(obstanz+10)*(i+10),0,i)
                     * osg::Matrix::rotate(2*M_PI/obstanz*i,0,0,1));
          global.obstacles.push_back(s);
        }
      }
      break;
    case labyrint:
      {
      double factor=2.5;
      playground = new ComplexPlayground(odeHandle, osgHandle,
                                         absolutePath + "labyrint2.fig", factor, 0.02);
      playground->setPosition(osg::Vec3(0,0,0.1));
      global.obstacles.push_back(playground);
      }
      break;
    case complexpl:
      {
      double factor=2;
      playground = new ComplexPlayground(odeHandle, osgHandle,
                                         absolutePath + "labyrint42.fig", factor, 0.05);


      //playground = new ComplexPlayground(odeHandle, osgHandle, "labyrint.fig", factor, 0.05);
      playground->setTexture("Images/wood.rgb");
      playground->setColor(Color(200/255.0,200/255.0,41/255.0,1));

      playground->setPosition(osg::Vec3(0,0,0.1));
      global.obstacles.push_back(playground);

    for(int i=0; i<2; i++){
      PassiveSphere* s =
        new PassiveSphere(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.5);
      s->setPosition(Pos(i*2, i-6, 1));
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);
    }

    for(int i=0; i<2; i++){
      PassiveBox* b =
        new PassiveBox(odeHandle,
                          osgHandle, osg::Vec3(0.4+i*0.2,0.4+i*0.2,0.4+i*0.2));
      b->setPosition(Pos(i  , i+6, 1));
      b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
      b->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(b);
    }

    for(int i=0; i<2; i++){
      PassiveCapsule* c =
        new PassiveCapsule(odeHandle, osgHandle, 0.2f, 0.3f, 0.3f);
      c->setPosition(Pos(i-5, -i-1, 1));
      c->setColor(Color(0.2f,0.2f,1.0f,0.5f));
      c->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(c);
    }


      }
      break;
    case none:
      playground=0;
      break;
    }



    //****************

    // create new robot
    //    OdeHandle sphereOdeHandle = odeHandle;
    //     sphereOdeHandle.substance.setCollisionCallback(sphereColl, 0);
    FourWheeledConf fwc = FourWheeled::getDefaultConf();
    fwc.irRangeFront=1.0; // the robot drives backwards ;-|
    fwc.irRangeSide=0.75;
    fwc.irRangeBack=2.0;
    fwc.irFront=true;
    fwc.irBack=true;
    fwc.irSide=true;
    OdeRobot* nimm4 = new FourWheeled ( odeHandle, osgHandle, fwc, "4Wheel_Reinf56");
    nimm4->addSensor(std::make_shared<Sensor>(SpeedSensor(1,SpeedSensor::TranslationalRel, Sensor::Z)));
    nimm4->addSensor(std::make_shared<Sensor>(SpeedSensor(1,SpeedSensor::RotationalRel, Sensor::X)));
    robot = nimm4;

    if(playgr==roundcorridor){
      robot->place(Pos(11,0,0.1));
      setCameraHomePos(Pos(8.73231, 16.2483, 4.53275),  Pos(-171.649, -14.6511, 0));
    }
    else{
      robot->place(Pos(0,0,0.1));
    }


    if(useMultiAgent){
      MultiReinforceConf msc = MultiReinforce::getDefaultConf();
      msc.tauE1=10;
      msc.tauH=5;
      msc.tauI=50;
      list<string> fs;
      {
        msc.numContext = 8;
        msc.useY=false;
        fs+=string(absolutePath + "4Wheel_30min_00.net");
        fs+=string(absolutePath + "4Wheel_30min_01.net");
        fs+=string(absolutePath + "4Wheel_30min_02.net");
        fs+=string(absolutePath + "4Wheel_30min_03.net");
        fs+=string(absolutePath + "4Wheel_30min_04.net");
        fs+=string(absolutePath + "4Wheel_30min_05.net");
        fs+=string(absolutePath + "4Wheel_30min_06.net");
        fs+=string(absolutePath + "4Wheel_30min_07.net");
      }
      msc.satFiles      = fs;

      msc.qlearning=qlearning;
      msc.useDerive=false;
      msc.reinforce_interval=interval;

      //controller = new ReinforceCmpStateIRController(msc);
      controller = new ReinforceIRController(msc);
    }else{
      ClassicReinforceConf crc = ClassicReinforce::getDefaultConf();
      crc.qlearning=qlearning;
      crc.numContext=8;
      crc.reinforce_interval=interval;
      controller = new ReinforceNimm4Controller(crc);
    }

    wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
    agent = new OdeAgent ( plotoptions );
    agent->init ( controller , robot , wiring );
    if(track) agent->setTrackOptions(TrackRobot(true,true,false,true,"cluttered",20));
    global.agents.push_back ( agent );

    global.configs.push_back(qlearning);
    global.configs.push_back(controller);


    log = fopen("qlearning.log","w");
    fprintf(log,"#C time eps discount expl interval sarsa avg col_rew\n");
  }

  virtual void end(GlobalData& globalData){
//     // store controller:
//     char filename[128];
//     sprintf(filename,"4Wheel_160min_%2.2f_%2.2f",);
//     FILE* f = fopen(filename,"wb");
//     if(f){
//       controller->store(f);
//       fclose(f);
//     }
    if(log) fclose(log);
  }


  virtual void addCallback(GlobalData& global, bool draw, bool pause, bool control) {
    if(rareprint){
      int times[8]={600,1200,2400,4800,9600,19200,38400,76800}; // 10 20 40 80 160 320 640 1280 minutes
      for(int i=0; i<8; i++){
        if(fabs(global.time-times[i])<0.005){
          fprintf(log,"%i %g %g %g %g %i %i %g\n",times[i],qlearning->getParam("eps"),
                  qlearning->getParam("discount"), qlearning->getParam("expl"),
                  controller->getParam("interval"), qlearning->useSARSA,
                  rewavg, qlearning->getCollectedReward());
        }
      }
    }else{
      if((int(global.time)%300)==0){
        if(fabs(((int)global.time)-global.time)<=0.01){
          printf("%i %g\n",((int)global.time),global.time);
          fprintf(log,"%i %g %g %g %g %i %i %g\n",(int)global.time,qlearning->getParam("eps"),
                  qlearning->getParam("discount"), qlearning->getParam("expl"),
                  controller->getParam("interval"), qlearning->useSARSA,
                  rewavg, qlearning->getCollectedReward());
        }
      }
    }
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
  playgr=longsquarecorridor;
  int index = Base::contains(argv, argc, "-pl");
  if(index && (argc > index)){
    if(argv[index][0]=='l') playgr=labyrint;
    else if(strcmp(argv[index],"42")==0) playgr=complexpl;
    else if(argv[index][0]=='r') playgr=roundcorridor;
    else if(argv[index][0]=='s') playgr=longsquarecorridor;
    else if(argv[index][0]=='c') playgr=cluttered;
    else if(argv[index][0]=='n') playgr=none;
  }
  index = Base::contains(argv, argc, "-ql");
  double eps=0.1,disc=0.7,expl=0.2;
  int sarsa=1;
  if(index && (argc > index+4)){
    eps=atof(argv[index]);
    disc=atof(argv[index+1]);
    expl=atof(argv[index+2]);
    sarsa=atoi(argv[index+3]);
    interval=atoi(argv[index+4]);
  }
  useMultiAgent = !Base::contains(argv, argc, "-classic");
  track         = Base::contains(argv, argc, "-t");
  rareprint     = Base::contains(argv, argc, "-rare");
  if(Base::contains(argv, argc, "-nographics")){
    absolutePath="/home/georg/sim/4Wheel_reinf/data/";
    realtimefactor=0;
  }

  index = Base::contains(argv, argc, "-rewavg");
  if(index && (argc > index))
    rewavg=atoi(argv[index]);

  int tau = int(rewavg*60*50.0/interval); // 5 minutes
  cout << tau;

  QLearning* qlearning = new QLearning(eps, disc, expl, 1, false, sarsa, tau);

  ThisSim sim(qlearning);
  sim.setCaption("Martius, Der, Herrmann");
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

