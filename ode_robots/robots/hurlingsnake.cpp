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

#include "hurlingsnake.h"
#include "osgprimitive.h"

using namespace std;

namespace lpzrobots {

  /**
   * Constructor
   * @param w world in which robot should be created
   * @param s space in which robot should be created
   * @param c contactgroup for collision treatment
   */
  HurlingSnake::HurlingSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                             const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), oldp(0,0,0){
    factorSensor=20.0;
    frictionGround=0.3;

    addParameterDef("factorForce", &factorForce,3.0, "factor of motor output");
    addParameterDef("factorSensor", &factorSensor,20.0, "factor of velocity as sensor value");
    addParameterDef("frictionGround", &frictionGround,0.3, "friction coefficient for ground");
    addParameterDef("frictionRoll", &frictionRoll,0.0, "friction coefficient for rolling");
    addParameterDef("place", &placedummy,0, "place the robot at 0,0");


    /* Parameter:
       gravity= -0.5
       factorForce=3.0;
       factorSensor=20.0;
       frictionGround=0.3;
       InvertMotorNStep():
       controller->setParam("steps", 2);
       controller->setParam("adaptrate", 0.001);
       controller->setParam("nomupdate", 0.001);

       gravity= -1
       factorForce=5.0;
       factorSensor=20.0;
       frictionGround=0.3;
       InvertMotorNStep():
       controller->setParam("steps", 2);
       controller->setParam("adaptrate", 0.001);
       controller->setParam("nomupdate", 0.001);



    */
    created=false;

    this->osgHandle.color=Color(1,1,0);

    NUM= 10;                /* number of spheres */

    //    SIDE= 0.2;                /* side length of a box */
    MASS= 1.0;                /* mass of a sphere*/
    RADIUS= 0.1732f;        /* sphere radius */

    sensorno = 2;
    motorno  = 2;

  };

  HurlingSnake::~HurlingSnake(){
    destroy();
  }


  void HurlingSnake::placeIntern(const osg::Matrix& pose){
    // lift the snake about its radius
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, RADIUS));
    create(p2);
  };


  void HurlingSnake::doInternalStuff(GlobalData& global){
    OdeRobot::doInternalStuff(global);
    // decellerate
    for (int i=0; i<NUM; i++) {
      objects[i]->decellerate(0,frictionRoll);
    }
  }


  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1]
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int HurlingSnake::getSensorsIntern(sensor* sensors, int sensornumber){
    int len = (sensornumber < sensorno)? sensornumber : sensorno;

    Pos p(objects[NUM-1]->getPosition());      //read actual position
    Pos s = (p - oldp)*factorSensor;

    sensors[0]=s.x();
    sensors[1]=s.y();
    oldp=p;
    return len;
  }


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void HurlingSnake::setMotorsIntern(const double* motors, int motornumber){
    //  dBodyAddForce (objects[NUM-1].body,motors[0]*factorForce,motors[1]*factorForce,motors[2]*factorForce);
    // force vector in global frame of reference
    dBodyAddForce (objects[NUM-1]->getBody(),motors[0]*factorForce,motors[1]*factorForce,0);
    // force vector is applied relative to the body's own frame of reference
    //dBodyAddRelForce (objects[NUM-1]->getBody(),motors[0]*factorForce,motors[1]*factorForce,0);
  }


  /** returns number of sensors
   */
  int HurlingSnake::getSensorNumberIntern(){
    return sensorno;
  }

  /** returns number of motors
   */
  int HurlingSnake::getMotorNumberIntern(){
    return motorno;
  }


  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments)
      @return length of the list
  */
  int HurlingSnake::getSegmentsPosition(std::vector<Position> &poslist){
    Position pos;
    for (int i=0; i<NUM; i++){
      Pos p = objects[i]->getPosition();
      poslist.push_back(p.toPosition());
    }
    return NUM;
  }



  void HurlingSnake::create(const osg::Matrix& pose){
    if (created){
      destroy();
    }
    // create vehicle space and add it to parentspace
    odeHandle.createNewSimpleSpace(parentspace,false);
    odeHandle.substance.roughness=frictionGround;

    joints.resize(NUM-1);
    objects.resize(NUM);

    for (int i=0; i<NUM; i++) {
      objects[i] = new Sphere(RADIUS);
      objects[i]->setTexture("Images/cross_stripes.rgb");
      //objects[i]->setTexture("Images/wood.rgb");
      if (i==NUM-1){
        OsgHandle osgHandle_head = osgHandle;
        osgHandle_head.color = Color(1, 0, 0);
        objects[i]->init(odeHandle, MASS, osgHandle_head);
      } else {
        objects[i]->init(odeHandle, MASS, osgHandle);
      }
      objects[i]->setPose(osg::Matrix::translate(i*RADIUS*2*1.1, 0, 0+0.03) * pose);
    }
    oldp = objects[NUM-1]->getPosition();
    for (int i=0; i<(NUM-1); i++) {
      Pos p1(objects[i]->getPosition());
      Pos p2(objects[i+1]->getPosition());
      joints[i] = new BallJoint(objects[i],objects[i+1], (p1+p2)/2);
      joints[i]->init(odeHandle, osgHandle, true, RADIUS/10, false);
    }

    created=true;
  }


  /** destroys robot
   */
  void HurlingSnake::destroy(){
    if (created){
      cleanup();
      odeHandle.deleteSpace();
    }
    created=false;
  }


  void HurlingSnake::notifyOnChange(const paramkey& key){
    if(key == "frictionGround") {
      // change substances
      for (int i=0; i<NUM; i++) {
        objects[i]->substance.roughness=frictionGround;
      }
    }
    else if(key == "place") {
      OdeRobot::placeIntern(TRANSM(0,0,3)) ;
    }
  }


}
