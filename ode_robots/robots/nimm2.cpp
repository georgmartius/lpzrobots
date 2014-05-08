/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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

#include <ode-dbl/ode.h>
#include <assert.h>
#include <osg/Matrix>
#include <selforg/controller_misc.h>

#include "nimm2.h"
#include "irsensor.h"
#include "osgprimitive.h"

using namespace osg;
using namespace std;


namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff, and default configuration
  Nimm2::Nimm2(const OdeHandle& odehandle, const OsgHandle& osgHandle,
               const Nimm2Conf& conf, const std::string& name)
    : OdeRobot(odehandle, osgHandle, name, "$Id$"), conf(conf) {
            contactPoints=0;

    // robot not created up to now
    created=false;

    // Nimm2 color ;-)
    this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);
    // can be overwritten in main.cpp of simulation with setColor

    // maximal used force is calculated from the force and size given in the configuration
    max_force   = conf.force*conf.size*conf.size;

    addParameter("speed", &this->conf.speed);
    addParameter("max_force", &max_force);

    height=conf.size;

    width=conf.size/2;  // radius of body
    radius=(width/2) * conf.wheelSize;  //radius of wheels
    wheelthickness=conf.size/10; // thickness of the wheels (if cylinder used, no spheres)
    cmass=4.0*conf.size*conf.massFactor;    // mass of body
    wmass=conf.size*conf.massFactor/5.0;  // mass of wheels
    if(conf.singleMotor){ //-> one dimensional robot
      sensorno=1;
      motorno=1;
    } else { // -> both wheels actuated independently
      sensorno=2;
      motorno=2;
    }

    if (conf.cigarMode){
      length=conf.size*conf.cigarLength;    // long body
      if(conf.wheelOffset<0) { // automatic mode
        wheeloffset= -length/4.0+radius+.1;  // wheels at the end of the cylinder, and the opposite endas the bumper
      }else{
        wheeloffset= -conf.wheelOffset*length;
      }
      // was wheeloffset= -length/4
      number_bumpers=2;        // if wheels not at center only one bumper
      max_force   = 2*conf.force*conf.size*conf.size;
    }
    else{
      length=conf.size/2;     // short body
      if(conf.wheelOffset<0) { // automatic mode
        wheeloffset=0.0;        // wheels at center of body
      }else{
        wheeloffset= -conf.wheelOffset*length;
      }
      number_bumpers=2;       // if wheels at center 2 bumpers (one at each end)
    }

    // increase sensornumber by 2 if front infrared sensors are used
    sensorno+= conf.irFront * 2;
    // increase sensornumber by 4 if side infrared sensors are used
    sensorno+= conf.irSide * 4;
    // increase sensornumber by 2 if rear infrared sensors are used
    sensorno+= conf.irBack * 2;

        visForce = conf.visForce;
        if (visForce) {
                sumForce=0;
        }
  };


  Nimm2::~Nimm2(){
    destroy();
  }


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Nimm2::setMotorsIntern(const double* motors, int motornumber){
    assert(created);
    assert(motornumber == motorno);
    if(conf.singleMotor){ // set the same motorcommand to both wheels
      joints[0]->setParam(dParamVel2, clip(motors[0],-1.,1.)*conf.speed); // set velocity
      joints[0]->setParam(dParamFMax2,max_force);            // set maximal force
      joints[1]->setParam(dParamVel2, clip(motors[0],-1.,1.)*conf.speed);
      joints[1]->setParam(dParamFMax2,max_force);
    } else {
      for (int i=0; i<2; i++){ // set different motorcommands to the wheels
        joints[i]->setParam(dParamVel2, clip(motors[i],-1.,1.)*conf.speed);
        joints[i]->setParam(dParamFMax2,max_force);
      }
    }
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Nimm2::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created);

    // choose sensornumber according to number of motors
    // - one motorcommand -> one sensorvalue
    // - motors indepently controlled -> two sensorvalues
    int len = conf.singleMotor ? 1 : 2;
    for (int i=0; i<len; i++){
      sensors[i]=dynamic_cast<Hinge2Joint*>(joints[i])->getPosition2Rate();  // readout wheel velocity
      sensors[i]/=conf.speed;  //scaling
    }
    return len;
  };


  void Nimm2::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, width*0.6));
    create(p2);

  };

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments)
      @return length of the list
  */
  int Nimm2::getSegmentsPosition(std::vector<Position> &poslist){
    assert(created);
    for (int i=0; i<3; i++){
      poslist.push_back(Position(dBodyGetPosition(objects[i]->getBody())));
    }
    return 3;
  };

  /**
   * updates the osg notes and sensorbank
   */
  void Nimm2::update() {
    OdeRobot::update();
    assert(created); // robot must exist
  }


  void Nimm2::doInternalStuff(GlobalData& globalData){
    OdeRobot::doInternalStuff(globalData);
    if (visForce) {
      sumForce=0;
      contactPoints=0;
    }
  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Nimm2::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    // robot will be inserted in the vehicle space
    odeHandle.createNewSimpleSpace(parentspace,true);
    objects.resize(3);
    joints.resize(2);


    OdeHandle wheelHandle(odeHandle);
    wheelHandle.substance.toRubber(40);
    wheelHandle.substance.slip=conf.wheelSlip;

    // create body
    // - create cylinder for main body (with radius and length)
    // - init cylinder with odehandle, mass and osghandle
    // - rotate and place body (here by 90 around the y-axis)
    // - set texture for cylinder
    // - put it into objects[0]

    if (conf.boxMode) {
      double dheight = 0.0;
      double height = width/4*3 + dheight;
      // height, width and length
      Box* box = new Box(height,conf.boxWidth*width/3, length/4*3);
      box->getOSGPrimitive()->setTexture("Images/wood.rgb");
      box->init(odeHandle, cmass*5, osgHandle);
      box->setPose(Matrix::rotate(M_PI/2, 0, 1, 0) * pose * Matrix::translate(0, 0, dheight/2));
      box->substance.toMetal(0);
      objects[0]=box;
    } else {
      Capsule* cap = new Capsule(width/2, length);
      cap->getOSGPrimitive()->setTexture("Images/wood.rgb");
      cap->init(odeHandle, cmass, osgHandle);
      cap->setPose(Matrix::rotate(M_PI/2, 0, 1, 0) * pose);
      objects[0]=cap;
    }

    // create bumper if required
    // - create cylinder with radius and length
    // - position bumper relative to main body
    //  (using transform object "glues" it together without using joints, see ODE documentation)
    // - init cylinder with odehandle, mass and osghandle
    if (conf.bumper && !conf.boxMode){
      for (int i=0; i<number_bumpers; i++){
        bumper[i].bump = new Capsule(width/4, 2*radius+width/2);
        bumper[i].trans = new Transform(objects[0], bumper[i].bump,
                                        Matrix::rotate(M_PI/2.0, Vec3(1, 0, 0)) *
                                        Matrix::translate(0, 0, i==0 ? -(length/2) : (length/2)));
        bumper[i].trans->init(odeHandle, 0, osgHandle);
        objects.push_back(bumper[i].trans);
      }
    } else if (conf.bumper && conf.boxMode){
      for (int i=0; i<number_bumpers; i++){
        bumper[i].bump = new Box(height/3,width/4, 2*radius+width/2);
        bumper[i].trans = new Transform(objects[0], bumper[i].bump,
                                        Matrix::rotate(M_PI/2.0, Vec3(1, 0, 0)) *
                                        Matrix::translate(0, 0, i==0 ? -(length/4) : (length/4)));
        bumper[i].trans->init(odeHandle, 0, osgHandle);
        bumper[i].bump->substance.toMetal(0);
        objects.push_back(bumper[i].trans);
      }
    }

    // create wheel bodies
    OsgHandle osgHandleWheels(osgHandle);    // new osghandle with color for wheels
    osgHandleWheels.color = Color(1.0,1.0,1.0);
    for (int i=1; i<3; i++) {
      if(conf.sphereWheels) { // for spherical wheels
        Sphere* wheel = new Sphere(radius);      // create spheres
        wheel->getOSGPrimitive()->setTexture(conf.wheelTexture); // set texture for wheels
        wheel->init(wheelHandle, wmass, osgHandleWheels); // init with odehandle, mass, and osghandle

        wheel->setPose(Matrix::rotate(M_PI/2.0, 1, 0, 0) *
                       Matrix::translate(wheeloffset,
                                         (i==2 ? -1 : 1) * (width*0.5+wheelthickness), 0) *
                       pose); // place wheels
        objects[i] = wheel;
        if (conf.boxMode) {
          //          wheel->substance.toRubber( 40.0);
           // wheel->substance.toSnow(0.0);
        }
      }else{ // for "normal" wheels
        Cylinder* wheel = new Cylinder(radius, wheelthickness);
        wheel->getOSGPrimitive()->setTexture("Images/tire.rgb"); // set texture for wheels
        wheel->init(wheelHandle, wmass, osgHandleWheels);
        wheel->setPose(Matrix::rotate(M_PI/2.0, Vec3(1,0,0)) *
                       Matrix::translate(wheeloffset,
                                         (i==2 ? -1 : 1) * (width*0.5+wheelthickness), 0)* pose);
        objects[i] = wheel;
      }
    }


    // set joints between wheels and body (see ODE documentation)
    // - create joint
    // - init joint
    // - set stop parameters
    for (int i=0; i<2; i++) {
      joints[i] = new Hinge2Joint(objects[0], objects[i+1], objects[i+1]->getPosition(),
                                 Axis(0, 0, 1)*pose, Axis(0, -1, 0)*pose);
      joints[i]->init(odeHandle, osgHandleWheels, true, conf.sphereWheels ? 2.01 * radius : wheelthickness*1.05 );
      // set stops to make sure wheels always stay in alignment
      joints[i]->setParam(dParamLoStop,0);
      joints[i]->setParam(dParamHiStop,0);
    }


    /* initialize sensorbank (for use of infrared sensors)
     * sensor values (if sensors used) are saved in the vector of
     * sensorvalues in the following order:
     * front left
     * front right
     * right front
     * right rear
     * rear rigth
     * rear left
     * left rear
     * left front
    */
    RaySensorBank* irSensorBank = new RaySensorBank();
    irSensorBank->setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
    double irpos;
    if(conf.boxMode){
      irpos = length*3.0/8.0 + width/60;
    } else {
      irpos = length/2 + width/2 - width/60;
    }
    if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, objects[0],
                                    Matrix::rotate(i*M_PI/10, Vec3(1,0,0)) *
                                    Matrix::translate(0,-i*width/10,irpos ),
                                    conf.irRange, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add right front and right rear infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor();
        if (conf.bumper){ // if bumpers used place on bumper
          irSensorBank->registerSensor(sensor, objects[0],
                                      //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                                      Matrix::rotate(M_PI/2, Vec3(1,0,0)) *
                                      Matrix::translate(0,-width,-i*(length/2) ),
                                      conf.irRange, RaySensor::drawAll);

        }else{ // place on body
          irSensorBank->registerSensor(sensor, objects[0],
                                      //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                                      Matrix::rotate(M_PI/2, Vec3(1,0,0)) *
                                      Matrix::translate(0,-width/2,i*(length/2) ),
                                      conf.irRange, RaySensor::drawAll);
        }
      }
    }
    if (conf.irBack){ // add rear right and rear left infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, objects[0],
                                    Matrix::rotate(-i*M_PI/10, Vec3(1,0,0)) *
                                    Matrix::rotate(i*M_PI, Vec3(0,1,0)) *
                                    Matrix::translate(0,i*width/10,-irpos ),
                                    conf.irRange, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add left rear and left front infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor();
        if (conf.bumper){ // if bumpers used place on bumper
          irSensorBank->registerSensor(sensor, objects[0],
                                      //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                                      Matrix::rotate(-M_PI/2, Vec3(1,0,0)) *
                                      Matrix::translate(0,width,i*(length/2) ),
                                      conf.irRange, RaySensor::drawAll);

        } else { // else place at body
          irSensorBank->registerSensor(sensor, objects[0],
                                      //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                                      Matrix::rotate(-M_PI/2, Vec3(1,0,0)) *
                                      Matrix::translate(0,width/2,i*(length/2) ),
                                      conf.irRange, RaySensor::drawAll);
        }
      }
    }
    if(irSensorBank->size()>0) addSensor(std::shared_ptr<Sensor>(irSensorBank));
    created=true;
  };


  /** destroys vehicle and space
   */
  void Nimm2::destroy(){
    if (created){
      for (int i=0; i<2; i++){
        //        if(bumper[i].bump) delete bumper[i].bump; is done by transform primitive
        if(bumper[i].trans) delete bumper[i].trans;
      }
      cleanup();
      odeHandle.deleteSpace();
    }
    created=false;
  }
/*
        std::list<Inspectable::iparamkey> Nimm2::getInternalParamNames() const{
                std::list<Inspectable::iparamkey> keylist;
                if (visForce) {
                        keylist+=std::string("SumForce");
                        std::cout << "returning: SumForce!" << std::endl;
                }
                std::cout << "beep name.";
                return keylist;
        }


        std::list<Inspectable::iparamval> Nimm2::getInternalParams() const {
                std::list<Inspectable::iparamval> vallist;
                if (visForce) {
                        vallist+=sumForce;
                        std::cout << "SumForce =" << sumForce << std::endl;
                }
                std::cout << "beep val.";
                return vallist;
        }
*/
}

