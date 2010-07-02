/***************************************************************************
 *   Copyright (C) 2010 by                                                 *
 *    Guillaume de Chambrier <s0672742@sms.ed.ac.uk>                       *
 *    martius@informatik.uni-leipzig.de                                    *
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
 **************************************************************************/

#include <cmath>
#include <assert.h>
#include <ode/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include sensors
#include <ode_robots/speedsensor.h>
#include <ode_robots/irsensor.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/twoaxisservo.h>

#include <ode_robots/mathutils.h>

// include header file
#include "hexapod.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  int t = 1;
  int c = 1;

  // constructor:
  // - give handle for ODE and OSG stuff
  Hexapod::Hexapod(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		   const HexapodConf& c, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "HexaPod 0.9"), conf(c)
  { 
    // robot is not created till now
    pos1d = new double[3];
    pos1 = new dReal();
    pos2 = new dReal();
    t = 0;
    massOfobject = new dMass();
    getPos1 = true;
    timeCounter = conf.T;
    energyOneStep = new double[1];
    energyOneStep[0] =  0.0;
    costOfTran = 0.0;
    created=false;
    check = false;
    recordGait=false;
    dones = new bool[6];
    conf.v[0] = 0;

    angles = new double[12];
    heights = new double[6];
    hcorrection = 0.20803;

    legContactArray = new Leg[6];
    this->osgHandle.color = Color(1.0, 1,1,1);
    legmass= (1.0 - conf.percentageBodyMass)*conf.mass/(conf.legNumber*2.0);

  };


  int Hexapod::getMotorNumber(){
    return  2*hipservos.size();
  };

  /* sets actual motorcommands
     @param motors motors scaled to [-1,1] 
     @param motornumber length of the motor array
  */
  void Hexapod::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist
    int len = min(motornumber, getMotorNumber())/2;

    for(int i = 0; i < len; i++){
      hipservos[i]->set(motors[2*i],motors[2*i+1]);
    }
  };

  int Hexapod::getSensorNumber(){
    return 2*hipservos.size() + irSensorBank.size();
  };

  /* returns actual sensorvalues
     @param sensors sensors scaled to [-1,1] (more or less)
     @param sensornumber length of the sensor array
     @return number of actually written sensors
  */
  int Hexapod::getSensors(sensor* sensors, int sensornumber){
    assert(created);
    int len = min(sensornumber, getSensorNumber() - irSensorBank.size())/2;

    for(int i = 0; i < len; i++){
      sensors[2*i]   = hipservos[i]->get1();
      sensors[2*i+1] = hipservos[i]->get2();
    }

    len = 2*len;



    if (conf.irFront || conf.irBack){
      len += irSensorBank.get(sensors+len, sensornumber-len);
    }

    return len;
  };


  void Hexapod::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    //    Matrix p2;
    //    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8)); 
    create(pose);
  };


  /**
   * updates the osg notes
   */
  void Hexapod::update(){
    assert(created); // robot must exist
  
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }

    irSensorBank.update();
  };

  double Hexapod::outwardMechanicalPower(const dReal *torques,const dReal *angularV){

    double mechanicalPower = 0.0;

    for(int i = 0; i < 3; i++){
      mechanicalPower += torques[i]*angularV[i];
    }

    if(mechanicalPower <= 0.0){
      mechanicalPower = 0.0;
    }

    return mechanicalPower;
  }

  double Hexapod::energyConsumpThroughtHeatLoss(const dReal *torques){
    return pow(torques[0] + torques[1] + torques[2],2);
  }

  double Hexapod::energyConsumption(){

    const dReal *torques;
    const dReal *angularV;
    double gamma = 0.005;
    double e = 0.0;

    for(unsigned int i = 0; i < legs.size(); i++){
      torques = dBodyGetTorque(legs[i]->getBody());
      angularV = dBodyGetAngularVel(legs[i]->getBody());
      e += outwardMechanicalPower(torques,angularV) + gamma*energyConsumpThroughtHeatLoss(torques);
    }
    return e;

  }

  double Hexapod::getMassOfRobot(){

    double totalMass = 0.0;

    for(unsigned int i = 0; i < objects.size(); i++){
      dBodyGetMass(objects[i]->getBody(),massOfobject);
      totalMass += massOfobject->mass;
    }
    return totalMass;
  }

  double Hexapod::costOfTransport(double E, double W, double V, double T){
    return E/(W*V*T);
  }

  double Hexapod::round(double num, int x){

    return  ceil( ( num * pow( 10,x ) ) - 0.49 ) / pow( 10,x );

  }
  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param global structure that contains global data from the simulation environment
  */
  void Hexapod::doInternalStuff(GlobalData& global){     
    irSensorBank.reset();

    energyOneStep[0] = energyConsumption();

    t = global.time;

    if(global.time <= timeCounter){
      E_t += energyOneStep[0];
    }

    if(getPos1){
      pos1 = dBodyGetPosition(trunk->getBody());
      pos1d[0] = pos1[0];
      pos1d[1] = pos1[1];
      pos1d[2] = pos1[2];
      getPos1 = false;
    }

    const dReal* velocity = dBodyGetLinearVel( trunk->getBody() );
    const double v = abs(velocity[0]);
    conf.v[0] = v;


    if(global.time >= timeCounter){

      pos2 = dBodyGetPosition(trunk->getBody());
      distance = sqrt(pow((pos2[0] - pos1d[0]),2) + pow((pos2[1] - pos1d[1]),2) + pow((pos2[2] - pos1d[2]),2));
      conf.v[0] = distance/conf.T;
      costOfTran = costOfTransport(E_t,getMassOfRobot(),conf.v[0],conf.T);
      //cout<< "cost of Transport: " << costOfTran << endl;
      timeCounter += conf.T;
      E_t = 0.0;
      getPos1 = true;
    }


    for(unsigned int i = 0; i < 6; i++){

      const dReal *position = dBodyGetPosition(legContactArray[i].bodyID);

      // cout<< dJointGetUniversalAngle1(joints[0]->getJoint()) * 180/M_PI  << endl;
      // cout<< dJointGetUniversalAngle2(joints[0]->getJoint())  * 180/M_PI<< endl;
      //  cout << dJointGetUniversalAngle1(legContactArray[i].joint) * 180/M_PI << endl;
      //  cout << dJointGetUniversalAngle2(legContactArray[i].joint) * 180/M_PI << endl;

      heights[i] = abs(round(position[2] -  hcorrection,3));
      angles[2*i]   = dJointGetUniversalAngle1(legContactArray[i].joint) * 180/M_PI ;
      angles[2*i+1] = dJointGetUniversalAngle2(legContactArray[i].joint) * 180/M_PI ;

    }

  }

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects 
   *  and the default routine)  else false (collision is passed to other objects and 
   *  (if not treated) to the default robjects.outine).
   */


  bool Hexapod::collisionCallback(void *data, dGeomID o1, dGeomID o2){

    const int NUM_CONTACTS = 8;
    dContact contacts[NUM_CONTACTS];
    int numCollisions = dCollide(o1, o2, NUM_CONTACTS, &contacts[0].geom, sizeof(dContact));

    //set all contacts to zero
    for(int j = 0; j < 6; j++) {
      conf.legContacts[j] = 0;
    }

    for(int i = 0; i < numCollisions; ++i)
      {
	dBodyID b1 =  dGeomGetBody(contacts[i].geom.g1);

	if(legContactArray[0].bodyID == b1){conf.legContacts[0] = 1; }
	if(legContactArray[1].bodyID == b1){conf.legContacts[1] = 4; }
	if(legContactArray[2].bodyID == b1){conf.legContacts[2] = 2; }
	if(legContactArray[3].bodyID == b1){conf.legContacts[3] = 5; }
	if(legContactArray[4].bodyID == b1){conf.legContacts[4] = 3; }
	if(legContactArray[5].bodyID == b1){conf.legContacts[5] = 6; }

      }

    /*

    // cout<< "t: " << t << "   timeC: " << timeCounter << endl;
    if((t + 0.01) >= timeCounter){
    if(recordGait){
    cout<<"in here" << endl;
    fprintf(f,"%d,%d,%d,%d,%d,%d,%g;",conf.legContacts[0],conf.legContacts[2],conf.legContacts[4],conf.legContacts[1],conf.legContacts[3],conf.legContacts[5],t);
    fprintf(f,"\n");
    check = true;
    }else if(check == true && recordGait == false){
    fprintf(f,"]\n");
    fclose(f);
    check = false;
    }
    }*/



    return false;
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void Hexapod::create( const osg::Matrix& pose ){
    if (created) {
      destroy();
    }

    odeHandle.createNewSimpleSpace(parentspace,false);
    OsgHandle osgHandleJ = osgHandle.changeColor(Color(1.0,0.0,0.0));
    UniversalServo* servo;
    FixedJoint* fixedJoint;
    
    // create body
    double twidth = conf.size / 1.5;
    double theight = conf.size / 4;
    trunk = new Box(conf.size, twidth, theight);
    trunk->setTexture("Images/toy_fur3.jpg");
    trunk->init(odeHandle, conf.mass*conf.percentageBodyMass, osgHandle);
    osg::Matrix trunkPos = osg::Matrix::translate(0,0,conf.legLength)*pose;
    trunk->setPose(trunkPos);
    objects.push_back(trunk);


    osg::Matrix m0 = pose;

    if(conf.irSensors == true){
      for(int i = -1; i < 2; i+=2){

	irbox = new Box(0.1,0.1,0.1);
	irbox->setTexture("Images/toy_fur3.jpg");
	irbox->init(odeHandle, 0.00001, osgHandle);
	irbox->setPose(osg::Matrix::rotate(M_PI/4,0,0,1) *osg::Matrix::translate(i*conf.size/2,0,theight/2)*trunkPos);
	objects.push_back(irbox);
	fixedJoint = new FixedJoint(trunk,irbox);
	fixedJoint->init(odeHandle, osgHandleJ, true, 0.4);
	joints.push_back(fixedJoint);
      }

      for(int i = -1; i < 2; i+=2){

	irbox = new Box(0.1,0.1,0.15);
	irbox->setTexture("Images/toy_fur3.jpg");
	irbox->init(odeHandle, 0.00001, osgHandle);
	irbox->setPose(osg::Matrix::translate(0,i*twidth/2,theight/2 + 0.05)*trunkPos);
	objects.push_back(irbox);
	fixedJoint = new FixedJoint(trunk,irbox);
	fixedJoint->init(odeHandle, osgHandleJ, true, 0.4);
	joints.push_back(fixedJoint);
      }


      irSensorBank.init(odeHandle, osgHandle);

      if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
     	IRSensor* sensor = new IRSensor();
     	irSensorBank.registerSensor(sensor, objects[2],
     				    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
     				    Matrix::translate(1*0.05,0,0),
     				    conf.irRangeFront, RaySensor::drawAll);
       	IRSensor* sensor2 = new IRSensor();
       	irSensorBank.registerSensor(sensor2, objects[2],
				    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
				    Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
				    Matrix::translate(0,-0.05,0),
				    conf.irRangeFront, RaySensor::drawAll);

      }
      if (conf.irBack){ // add front left and front right infrared sensor to sensorbank if required

      	IRSensor* sensor = new IRSensor();
       	irSensorBank.registerSensor(sensor, objects[1],
       				    Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
       				    Matrix::translate(-1*0.05,0,0),
       				    conf.irRangeBack, RaySensor::drawAll);

	IRSensor* sensor2 = new IRSensor();
	irSensorBank.registerSensor(sensor2, objects[1],
				    Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
				    Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
				    Matrix::translate(0,0.05,0),
				    conf.irRangeBack, RaySensor::drawAll);
      }
      if(conf.irLeft){
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, objects[3],
				    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
				    Matrix::rotate(-M_PI/2, Vec3(0,0,1)) *
				    Matrix::translate(0,-0.05,0.05),
				    conf.irRangeLeft, RaySensor::drawAll);

    	/* IRSensor* sensor2 = new IRSensor();
	   irSensorBank.registerSensor(sensor2, objects[3],
	   Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
	   Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
	   Matrix::translate(0,-0.05,0),
	   conf.irRangeLeft, RaySensor::drawAll);*/
      }
      if(conf.irRight){
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, objects[4],
				    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
				    Matrix::rotate(M_PI/2, Vec3(0,0,1)) *
				    Matrix::translate(0,0.05,0.05),
				    conf.irRangeLeft, RaySensor::drawAll);


	/* IRSensor* sensor2 = new IRSensor();
	   irSensorBank.registerSensor(sensor2, objects[4],
	   Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
	   Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
	   Matrix::translate(0,0.05,0),
	   conf.irRangeRight, RaySensor::drawAll);*/
      }
    }


    // legs  (counted from back to front)
    double legdist = conf.size*0.9 / (conf.legNumber/2-1);
    for ( int n = 0; n < conf.legNumber; n++ ) {

      int v = n;

      double l1 = conf.legLength*0.5;
      double t1 = conf.legLength/10;
      double l2 = conf.legLength*0.5;
      double t2 = conf.legLength/10;


      // upper limp
      Primitive* coxaThorax;
      Pos pos = Pos(-conf.size/(2+0.2) + ((int)n/2) * legdist, n%2==0 ? - twidth/2 : twidth/2, conf.legLength - theight/3);

      osg::Matrix m = osg::Matrix::rotate(M_PI/2,v%2==0 ? -1 : 1,0,0) * osg::Matrix::translate(pos) * pose;
      coxaThorax = new Capsule(t1, l1);
      coxaThorax->setTexture("Images/toy_fur3.jpg");
      coxaThorax->init(odeHandle, legmass, osgHandle);

      osg::Matrix m1 =  osg::Matrix::translate(0,0,-l1/2) * osg::Matrix::rotate(M_PI,0,0,v%2==0 ? -1 : 1) * osg::Matrix::rotate(2*M_PI,0,v%2==0 ? -1 : 1,0) * m;

      coxaThorax->setPose(m1);
      thoraxPos.push_back(coxaThorax->getPosition());

      thorax.push_back(coxaThorax);

      objects.push_back(coxaThorax);
      legs.push_back(coxaThorax);
      // powered hip joint
      Pos nullpos(0,0,0);


      UniversalJoint* j = new UniversalJoint(trunk, coxaThorax, nullpos * m, osg::Matrix::rotate(M_PI,0,0,v%2==0 ? -1 : 1) * Axis(v%2==0 ? -1 : 1,0,0) * m,
					     osg::Matrix::rotate(M_PI,0,0,v%2==0 ? -1 : 1) *	Axis(0,1,0) * m);



      j->init(odeHandle, osgHandle, true, t1 * 2.1);
      joints.push_back(j);

      legContactArray[n].joint = j->getJoint();



      servo =  new UniversalServo(j,-conf.coxaJointLimitV, conf.coxaJointLimitV,conf.coxaPower,
				  -conf.coxaJointLimitH, conf.coxaJointLimitH,conf.coxaPower,conf.coxaThoraxDamping);
      hipservos.push_back(servo);



      Primitive* tibia;
      tibia = new Capsule(t2, l2);
      tibia->setTexture("Images/toy_fur3.jpg");
      tibia->init(odeHandle, legmass, osgHandle);
      osg::Matrix m2 =   osg::Matrix::translate(0,0,-l2/2)* osg::Matrix::rotate(1.5,v%2==0 ? -1 : 1,0,0) * osg::Matrix::translate(0,0,-l1/2) * m1;
      tibia->setPose(m2);
      objects.push_back(tibia);
      legs.push_back(tibia);

      legContactArray[n].legID = n;
      legContactArray[n].geomid = tibia->getGeom();
      legContactArray[n].bodyID = tibia->getBody();



      // powered knee joint
      FixedJoint* k = new FixedJoint(tibia,coxaThorax);
      k->init(odeHandle, osgHandle, true, t1 * 2.1);
      /*  HingeJoint* k = new HingeJoint(coxaThorax, tibia, Pos(0,0,-l1/2) * m1, Axis(v%2==0 ? -1 : 1,0,0) * m1);
	  k->init(odeHandle, osgHandle, true, t1 * 2.1);
	  k->setParam(dParamLoStop,0);
	  k->setParam(dParamHiStop,0);
	  k->*/
      joints.push_back(k);
      // lower limp should not collide with body!
      odeHandle.addIgnoredPair(trunk,tibia);



    }
    
    created=true;
  }; 


  /** destroys vehicle and space
   */
  void Hexapod::destroy(){
    if (created){
      //  odeHandle.removeIgnoredPair(bigboxtransform);
      odeHandle.removeIgnoredPair(trunk,headtrans);
      irSensorBank.clear();



      FOREACH(vector<UniversalServo*>, hipservos, i){
      }
      hipservos.clear();


      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
	if(*i) delete *i;
      }
      joints.clear();      
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
	if(*i) delete *i;
      }
      objects.clear();

      odeHandle.deleteSpace();
    }


    created=false;
  }



  /** The list of all parameters with there value as allocated lists.
   */
  Configurable::paramlist Hexapod::getParamList() const{
    paramlist list;
    list += pair<paramkey, paramval> (string("coxaPower"),   	conf.coxaPower);
    list += pair<paramkey, paramval> (string("coxaThorax"),   	conf.coxaThoraxDamping);
    list += pair<paramkey, paramval> (string("coxaJointLimitH"), conf.coxaJointLimitH);
    list += pair<paramkey, paramval> (string("coxaJointLimitV"), conf.coxaJointLimitV);
    //   list += pair<paramkey, paramval> (string("tebiaPower"),  	conf.tebiaPower);
    list += pair<paramkey, paramval> (string("tebiaCoxaDamping"),conf.tebiaCoxaDamping);
    list += pair<paramkey, paramval> (string("tebiaJointLimit"),conf.tebiaJointLimit);

    return list;
  }
  
  
  Configurable::paramval Hexapod::getParam(const paramkey& key) const{    
    if(key == "coxaPower") return conf.coxaPower;
    else if(key == "coxaThoraxDamping") return conf.coxaThoraxDamping;
    //  else if(key == "tebiaPower") return conf.tebiaPower;
    else if(key == "tebiaCoxaDamping") return conf.tebiaCoxaDamping;
    else if(key == "coxaJointLimitH") return conf.coxaJointLimitH;
    else if(key == "coxaJointLimitV") return conf.coxaJointLimitV;
    else if(key == "tebiaJointLimit") return conf.tebiaJointLimit;
    else  return Configurable::getParam(key) ;
  }
  
  bool Hexapod::setParam(const paramkey& key, paramval val){    
    if(key == "coxaPower") {
      conf.coxaPower = val;

      FOREACH(vector<UniversalServo*>, hipservos, i){
	if(*i){
	  (*i)->setPower1(conf.coxaPower);
	  (*i)->setPower2(conf.coxaPower);
	}
      }

    } else if(key == "coxaThoraxDamping") {
      conf.coxaThoraxDamping = val;


      FOREACH(vector<UniversalServo*>, hipservos, i){
	if(*i) {
	  (*i)->damping1() = conf.coxaThoraxDamping;
	  (*i)->damping2() = conf.coxaThoraxDamping;
	}
      }

    } else if(key == "coxaJointLimitH") {
      conf.coxaJointLimitH = val;


      FOREACH(vector<UniversalServo*>, hipservos, i){
	if(*i){
	  (*i)->setMinMax1(-val,+val);
	  (*i)->setMinMax2(-val,+val);
	}
      }

    }  else return Configurable::setParam(key, val);
    return true;
  }

}
