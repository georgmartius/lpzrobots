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
#include <osg/Matrix>
#include "mathutils.h"
#include "hand.h"
#include <osgprimitive.h>

//using namespace std;

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff and default configuration
  Hand::Hand(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const HandConf& conf, const std::string& name)
    // calling OdeRobots construtor with name of the actual robot
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(conf), oldp(0,0,0){

    // setting parameters
    frictionGround=0.3;
    velocity=2;

    created=false;

    this->osgHandle.color=Color(0,1,1);


    addParameter("servo_motor_Power", &this->conf.servo_motor_Power, 0,100);
    addParameter("factorSensor", &this->conf.factorSensor,0,01);
    addParameter("irRange",&this->conf.irRange,0,10);

    if (conf.one_finger_as_one_motor){  // one finger as one motor
      sensorno=6;
      motorno=6;
    } else { // each finger joint as a seperate motor
      sensorno=15;
      motorno=15;
    }

    if (!conf.fix_palm_joint){ // if joint between palm and forearm is not fixed
      sensorno += 2;
      motorno  += 2;
    }

    if (conf.ir_sensor_used){ // if infrared sensors are used
      if (conf.irs_at_fingerbottom){
        sensorno+=5;
      }
      if (conf.irs_at_fingercenter){
        sensorno+=5;
      }
      if (conf.irs_at_fingertop){
        sensorno+=5;
      }
      if (conf.irs_at_fingertip){
        // TODO: reset to 5 after testing of InvertNChannelControllerHebbH
        //        sensorno+=5;
        sensorno+=6;
      }
    }


  };


  void Hand::update() {
    OdeRobot::update();
    assert(created); // robot must exist





    if(contact_joint_created){
      for (std::vector<OSGPrimitive*>::iterator i = osg_objects.begin(); i!=  osg_objects.end(); i++){
        if(*i) delete *i;
      }
      osg_objects.clear();
      contact_joint_created=false;
    }

    // update (draw) sensorbank with infrared sensors
    if (conf.ir_sensor_used){
      irSensorBank.update();
    }


  }

  void Hand::placeIntern(const osg::Matrix& pose){
    create(pose);
  };

  void Hand::sense(GlobalData& globalData) {
    // reset ir sensors to maximum value
    irSensorBank.sense(globalData);
    OdeRobot::sense(globalData);
  }

  // internal collisions
  // todo: wird im moment nich gebraucht, kann es ganz raus?
  void Hand::mycallback(void *data, dGeomID o1, dGeomID o2){

    // connected bodies are allowed to intersect
    // if( dAreConnected(dGeomGetBody(o1), dGeomGetBody(o2)) )
    // return;


    //if(o1 == (dGeomID)odeHandle.space) me->irSensorBank.sense(o2);
    //if(o2 == (dGeomID)odeHandle.space) me->irSensorBank.sense(o1);

    Hand* me = (Hand*)data;


    int i=0;
    int o1_index= -1;
    int o2_index= -1;
    for (std::vector<Primitive*>::iterator n = me->objects.begin(); n!= me->objects.end(); n++, i++){
      if( (*n)->getGeom() == o1)
        o1_index=i;
      if( (*n)->getGeom() == o2)
        o2_index=i;
    }

    if(o1_index >= 0 && o2_index >= 0){

      int n;
      const int N = 10;
      dContact contact[N];
      n=dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      if(n >0){
        for (int i=0; i<n; i++) {
          contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
            dContactSoftERP | dContactSoftCFM | dContactApprox1;
          contact[i].surface.mu = dInfinity;
          contact[i].surface.slip1 = 0.1;
          contact[i].surface.slip2 = 0.1;
          contact[i].surface.soft_erp = 0.5;
          contact[i].surface.soft_cfm = 0.3;
          dJointID c = dJointCreateContact( me->odeHandle.world, me->odeHandle.jointGroup, &contact[i]);
          dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;

          //    for (std::vector<Primitive*>::iterator u = me->objects.begin()+2; u!= me->objects.end(); u++ ){
          for (unsigned int u=2; u < me->objects.size(); u++){
            if((contact[i].geom.g1 == me->objects[1]->getGeom() || contact[i].geom.g2 == me->objects[1]->getGeom() )
               && (contact[i].geom.g1 == me->objects[u]->getGeom() || contact[i].geom.g2 == me->objects[u]->getGeom() ) )
              {
                contact[i].geom.depth=5;
              }

            /*
            //in this just not let the ir-sensors to enter in the palm
            if (me->conf.ir_sensor_used == true){
              if ((contact[i].geom.g1 == me->objects[1]->getGeom()   || contact[i].geom.g2 == me->objects[1]->getGeom())
                  && (contact[i].geom.g1 == me->ir_sensors[u-2]->getGeomID() || contact[i].geom.g2 ==  me->ir_sensors[u-2]->getGeomID() ) )
                {
                  contact[i].geom.depth=15;
                }
            }
            */
          }
          /*
            for (unsigned int u=2; u < me->objects.size(); u++){
            //(contact[i].geom.g1 == me->objects[1]->getGeom() || contact[i].geom.g2 == me->objects[1]->getGeom() )
            if (((contact[i].geom.g1 == me->objects[1]->getGeom() || contact[i].geom.g2 == me->objects[1]->getGeom()) || (contact[i].geom.g1 == me->objects[0]->getGeom() || contact[i].geom.g2 == me->objects[0]->getGeom())||
            contact[i].geom.g1 == me->objects[u]->getGeom() || contact[i].geom.g2 == me->objects[u]->getGeom()) && (contact[i].geom.g1 == me->ir_sensors[u-2]->getGeomID() || contact[i].geom.g2 ==  me->ir_sensors[u-2]->getGeomID()))
            {
            contact[i].geom.depth=0.0005;
            }

            }
          */
          //|| (contact[i].geom.g1 == me->ir_sensors[u-2]->getGeomID() || contact[i].geom.g2 ==  me->ir_sensors[u-2]->getGeomID()))
          //|| contact[i].geom.g1 == me->irSensorBank.sense(o2)->getGeomID() || contact[i].geom.g2 == me->irSensorBank.sense(o2)->getGeomID()
        }
      }

    }
  }


  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1]
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Hand::getSensorsIntern(sensor* sensors, int sensornumber){
    assert (sensorno == sensornumber);
    //   int len = (sensornumber < sensorno)? sensornumber : sensorno;
    //int len = min(sensornumber/2, (int)joints.size()+2);

    int sensorindex=0;
    if (!conf.fix_palm_joint){
      sensors[sensorindex]=((AngularMotor*)palm_motor_joint)->getParam(dParamVel);
      sensorindex++;
      sensors[sensorindex]=((AngularMotor*)palm_motor_joint)->getParam(dParamVel2);
      sensorindex++;
    }

    sensors[sensorindex]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel);
    sensorindex++;

    /* not usable in the actual setup, thumb_motor_joint has only 1 controllable DOF
    sensors[sensorindex]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel2);
    sensorindex++;
    */
    sensors[sensorindex]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel3);
    sensorindex++;


    if (conf.one_finger_as_one_motor){ // motors at one finger get the same value

      //Without_servo_motor
      // thumb already red out above

      // sensorvalues for index finger
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) palm_index)->getPosition1Rate();
      sensorindex++;
      // sensorvalues for middle finger
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) palm_middle)->getPosition1Rate();
      sensorindex++;
      //motorvalues for ring finger
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) palm_ring)->getPosition1Rate();
      sensorindex++;
      //motorvalues for little finger
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) palm_little)->getPosition1Rate();
      sensorindex++;

    } else { // each joint has its own sensor/motor value

      //Without_servo_motor
      //thumb
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) thumb_bt)->getPosition1Rate();
      sensorindex++;
      // index finger
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) palm_index)->getPosition1Rate();
      sensorindex++;
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) index_bc)->getPosition1Rate();
      sensorindex++;
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) index_ct)->getPosition1Rate();
      sensorindex++;
      // middle finger
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) palm_middle)->getPosition1Rate();
      sensorindex++;
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) middle_bc)->getPosition1Rate();
      sensorindex++;
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) middle_ct)->getPosition1Rate();
      sensorindex++;
      //ring finger
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) palm_ring)->getPosition1Rate();
      sensorindex++;
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) ring_bc)->getPosition1Rate();
      sensorindex++;
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) ring_ct)->getPosition1Rate();
      sensorindex++;
      //little finger
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) palm_little)->getPosition1Rate();
      sensorindex++;
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) little_bc)->getPosition1Rate();
      sensorindex++;
      sensors[sensorindex] = conf.factorSensor * ((HingeJoint*) little_ct)->getPosition1Rate();
      sensorindex++;
    }



    /*
    if (conf.one_finger_as_one_motor){ // only values of palm-finger joints sensed
      for (uint i = 4; i < joints.size()-1; i+=3){
        switch(conf.set_typ_of_motor){
        case(With_servo_motor):
          sensors[sensorindex] =  servos[i-2]->get();
          sensorindex++;
          break;
        case(Without_servo_motor):
          sensors[sensorindex] = conf.factorSensor * ((HingeJoint*)joints[i])->getPosition1Rate();
          sensorindex++;
          break;
        default:
          sensors[sensorindex] = conf.factorSensor * ((HingeJoint*)joints[i])->getPosition1Rate();
          sensorindex++;
          break;
        }
      }
    }else{ // value of each joint is sensed
      //   for (int i = 2; i < sensor_number; i++){
      for (uint i = 2; i < joints.size()-1; i++){
        switch(conf.set_typ_of_motor){
        case(With_servo_motor):
          sensors[sensorindex] =  servos[i-2]->get();
          sensorindex++;
          break;
        case(Without_servo_motor):
          sensors[sensorindex] = conf.factorSensor * ((HingeJoint*)joints[i])->getPosition1Rate();
          sensorindex++;
          break;
        default:
          sensors[sensorindex] = conf.factorSensor * ((HingeJoint*)joints[i])->getPosition1Rate();
          sensorindex++;
          break;
        }
      }
    }
    */
    if(conf.ir_sensor_used)
      {
        sensorindex += irSensorBank.get(sensors+sensorindex, sensorno-sensorindex);
      }
    return sensorindex;
   }


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Hand::setMotorsIntern(const double* motors, int motornumber){
    assert (created);
    assert (motorno == motornumber);

    //  todo: add servomotor control

    int motorindex=0;
    // setting the motor values for joint between palm and forarm (only when joint is not fixed)
    if (!conf.fix_palm_joint){
      ((AngularMotor3AxisEuler*)palm_motor_joint)->set(0, motors[motorindex]* velocity);
      motorindex++;
      ((AngularMotor3AxisEuler*)palm_motor_joint)->set(1, motors[motorindex]* velocity);
      motorindex++;
      ((AngularMotor3AxisEuler*)palm_motor_joint)->setPower(conf.power);
    }

    // setting the motor values for joint between palm and thumb
    ((AngularMotor3AxisEuler*)thumb_motor_joint)->set(0, motors[motorindex]* velocity);
    motorindex++;
    /* not used here
    ((AngularMotor3AxisEuler*)thumb_motor_joint)->set(1, motors[motorindex]* velocity);
    motorindex++;
    */
    ((AngularMotor3AxisEuler*)thumb_motor_joint)->set(2, motors[motorindex]* velocity);
    motorindex++;


    if (conf.one_finger_as_one_motor){ // motors at one finger get the same value
      /* fingers coupled * /
      motorindex--; // remaining joints of thumb shoud get the same motorvalue
      //Without_servo_motor
      ((HingeJoint*)thumb_bt) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)thumb_bt) -> setParam(dParamFMax, conf.power);

      motorindex++; //motorvalues for index finger
      ((HingeJoint*)palm_index) -> setParam ( dParamVel ,
                                              (motors[motorindex]+0.2*motors[motorindex+1]) * velocity );
      ((HingeJoint*)palm_index) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)index_bc) -> setParam ( dParamVel ,
                                            (motors[motorindex]+0.2*motors[motorindex+1])* velocity );
      ((HingeJoint*)index_bc) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)index_ct) -> setParam ( dParamVel ,
                                            (motors[motorindex]+0.2*motors[motorindex+1])* velocity );
      ((HingeJoint*)index_ct) -> setParam(dParamFMax, conf.power);

      motorindex++; //motorvalues for middle finger
      ((HingeJoint*)palm_middle) -> setParam ( dParamVel ,
                                               (motors[motorindex]
                                                +0.2*motors[motorindex-1]
                                                +0.2*motors[motorindex+1])* velocity );
      ((HingeJoint*)palm_middle) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)middle_bc) -> setParam ( dParamVel ,
                                             (motors[motorindex]
                                              +0.2*motors[motorindex-1]
                                              +0.2*motors[motorindex-+1])* velocity );
      ((HingeJoint*)middle_bc) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)middle_ct) -> setParam ( dParamVel ,
                                             (motors[motorindex]
                                              +0.2*motors[motorindex-1]
                                              +0.2*motors[motorindex+1])* velocity );
      ((HingeJoint*)middle_ct) -> setParam(dParamFMax, conf.power);

      motorindex++; //motorvalues for ring finger
      ((HingeJoint*)palm_ring) -> setParam ( dParamVel ,
                                             (motors[motorindex]
                                              +0.3*motors[motorindex-1]
                                              +0.1*motors[motorindex+1])* velocity );
      ((HingeJoint*)palm_ring) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)ring_bc) -> setParam ( dParamVel ,
                                           (motors[motorindex]
                                            +0.3*motors[motorindex-1]
                                            +0.1*motors[motorindex+1])* velocity );
      ((HingeJoint*)ring_bc) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)ring_ct) -> setParam ( dParamVel ,
                                           (motors[motorindex]
                                            +0.3*motors[motorindex-1]
                                            +0.1*motors[motorindex+1])* velocity );
      ((HingeJoint*)ring_ct) -> setParam(dParamFMax, conf.power);
      motorindex++; //motorvalues for little finger
      ((HingeJoint*)palm_little) -> setParam ( dParamVel ,
                                               (motors[motorindex]+0.2*motors[motorindex-1])* velocity );
      ((HingeJoint*)palm_little) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)little_bc) -> setParam ( dParamVel ,
                                             (motors[motorindex]+0.2*motors[motorindex-1])* velocity );
      ((HingeJoint*)little_bc) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)little_ct) -> setParam ( dParamVel ,
                                             (motors[motorindex]+0.2*motors[motorindex-1])* velocity );
      ((HingeJoint*)little_ct) -> setParam(dParamFMax, conf.power);

      */
      /* fingers not coupled */
      motorindex--; // remaining joints of thumb shoud get the same motorvalue
      //Without_servo_motor
      ((HingeJoint*)thumb_bt) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)thumb_bt) -> setParam(dParamFMax, conf.power);
      motorindex++; //motorvalues for index finger
      ((HingeJoint*)palm_index) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)palm_index) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)index_bc) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)index_bc) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)index_ct) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)index_ct) -> setParam(dParamFMax, conf.power);
      motorindex++; //motorvalues for middle finger
      ((HingeJoint*)palm_middle) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)palm_middle) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)middle_bc) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)middle_bc) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)middle_ct) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)middle_ct) -> setParam(dParamFMax, conf.power);
      motorindex++; //motorvalues for ring finger
      ((HingeJoint*)palm_ring) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)palm_ring) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)ring_bc) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)ring_bc) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)ring_ct) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)ring_ct) -> setParam(dParamFMax, conf.power);
      motorindex++; //motorvalues for little finger
      ((HingeJoint*)palm_little) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)palm_little) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)little_bc) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)little_bc) -> setParam(dParamFMax, conf.power);
      ((HingeJoint*)little_ct) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)little_ct) -> setParam(dParamFMax, conf.power);
      /**/

    } else { // each joint has its own motorvalue

      //Without_servo_motor
      //thumb
      ((HingeJoint*)thumb_bt) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)thumb_bt) -> setParam(dParamFMax, conf.power);
      motorindex++;
      // index finger
      ((HingeJoint*)palm_index) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)palm_index) -> setParam(dParamFMax, conf.power);
      motorindex++;
      ((HingeJoint*)index_bc) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)index_bc) -> setParam(dParamFMax, conf.power);
      motorindex++;
      ((HingeJoint*)index_ct) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)index_ct) -> setParam(dParamFMax, conf.power);
      motorindex++;
      // middle finger
      ((HingeJoint*)palm_middle) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)palm_middle) -> setParam(dParamFMax, conf.power);
      motorindex++;
      ((HingeJoint*)middle_bc) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)middle_bc) -> setParam(dParamFMax, conf.power);
      motorindex++;
      ((HingeJoint*)middle_ct) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)middle_ct) -> setParam(dParamFMax, conf.power);
      motorindex++;

      //ring finger
      ((HingeJoint*)palm_ring) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)palm_ring) -> setParam(dParamFMax, conf.power);
      motorindex++;
      ((HingeJoint*)ring_bc) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)ring_bc) -> setParam(dParamFMax, conf.power);
      motorindex++;
      ((HingeJoint*)ring_ct) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)ring_ct) -> setParam(dParamFMax, conf.power);
      motorindex++;

      //little finger
      ((HingeJoint*)palm_little) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)palm_little) -> setParam(dParamFMax, conf.power);
      motorindex++;
      ((HingeJoint*)little_bc) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)little_bc) -> setParam(dParamFMax, conf.power);
      motorindex++;
      ((HingeJoint*)little_ct) -> setParam ( dParamVel , motors[motorindex]* velocity );
      ((HingeJoint*)little_ct) -> setParam(dParamFMax, conf.power);
    }

    /*

    for (uint i = 2; i < joints.size()-1; i++){ // when using thumb joints
        //for (uint i = 4; i < joints.size()-1; i++){ // when not using thumb joints
        switch(conf.set_typ_of_motor){
        case(Without_servo_motor):
          ((HingeJoint*)joints[i])->setParam ( dParamVel , motors[motorindex]* velocity );
          ((HingeJoint*)joints[i])->setParam(dParamFMax, conf.power);
          break;
        case(With_servo_motor):
          servos[i-2]->set(motors[motorindex]);
          servos[i-2]->setPower(conf.servo_motor_Power);
          break;
        default:
          ((HingeJoint*)joints[i])->setParam ( dParamVel , motors[motorindex]* velocity );
          ((HingeJoint*)joints[i])->setParam(dParamFMax, conf.power);
          break;
        }
        if (i ==  3) motorindex++;  // change to motor value for index finger
        if (i ==  6) motorindex++;  // change to motor value for middle finger
        if (i ==  9) motorindex++;  // change to motor value for ring finger
        if (i == 12) motorindex++;  // change to motor value for little finger
      }
    }else{ // each motor gets its own value
      // joints.size()-1 since fixed joint is no motor
      for (uint i = 2; i < joints.size()-1; i++){
        switch(conf.set_typ_of_motor){
        case(Without_servo_motor):
          ((HingeJoint*)joints[i])->setParam ( dParamVel , motors[motorindex]* velocity );
          motorindex++;
          ((HingeJoint*)joints[i])->setParam(dParamFMax, conf.power);
          break;
        case(With_servo_motor):
          servos[i-2]->set(motors[motorindex]);
          motorindex++;
          servos[i-2]->setPower(conf.servo_motor_Power);
          break;
        default:
          ((HingeJoint*)joints[i])->setParam ( dParamVel , motors[motorindex]* velocity );
          motorindex++;
          ((HingeJoint*)joints[i])->setParam(dParamFMax, conf.power);
          break;
        }
      }
    }

    */
  }

  /** returns number of sensors
   */
  int Hand::getSensorNumberIntern(){
    return sensorno;
  }

  /** returns number of motors
   */
  int Hand::getMotorNumberIntern(){
    return motorno;
  }


  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments)
      @return length of the list
  */
  /*
    int Hand::getSegmentsPosition(vector<Position> &poslist){
    int number_objects=0;// or also with objects.size()
    Position pos;
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++                     ){
    if(*i) { Pos p = (*i)->getPosition();
    poslist.push_back(p.toPosition());
    ++number_objects;
    }
    }

    return number_objects; // return objects.size();
    }

  */

  void Hand::create(const osg::Matrix& pose){
    if (created){
      destroy();
    }

    // todo introduce gripmodes ?
    //  gripmode=lateral;

    // create vehicle space and add it to parentspace
    odeHandle.space = dSimpleSpaceCreate (parentspace);

    irSensorBank.setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
    irSensorBank.init(0);


    double forearm_length=5;
    double forearm_radius=0.5;
    double palm_radius=1.0;
    double palm_length=0.3; // (or better height)

    // palm created here
    // (because it is the main component,
    // placing the hand at (0,0,0) means the palm will be at (0,0,forearm_radius))
    // but added to objects after forearm to keep sequence of objects
    // which is up to now used in collision stuff

    // todo: cylinder is penetrable at the edges: make some workaround
    Primitive* palm = new Cylinder(palm_radius, palm_length);
    palm-> init (odeHandle , 0.1 , osgHandle);
    palm->setPose(osg::Matrix::translate(0, 0, forearm_radius) * pose);


    //--------------------forearm--------------------------------------------------
    Primitive*  forearm = new Capsule(forearm_radius, forearm_length);
    //  forearm->getOSGPrimitive()->setTexture("Images/finger_handflaeche2.png");
    forearm->init ( odeHandle, 1, osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
    forearm->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0)
                        *osg::Matrix::translate( -(forearm_length/2 + palm_radius) ,0 , 0)
                        *palm->getPose());
    objects.push_back(forearm);



    //--------------------palm------------------------------------------------------

    // todo: cylinder is penetrable at the edges: make some workaround
    // palm created above, but added to objects here to keep sequence (see above)
    objects.push_back(palm);


    // box inside the palm
    // because cylinder is penetrable
    /*
    Primitive* palm_box = new Box(0.8,1.3,0.3);
    Primitive* box_in_cylinder_palm = new Transform(objects[1],palm_box,
                                                    osg::Matrix::translate(0.05, 0, 0));
    box_in_cylinder_palm -> init (odeHandle , 0 , osgHandle);
    */



    //-------------------BallJoint between forearm and palm-------------------------

    Joint* forearm_palm = new BallJoint(forearm, palm, Pos( -palm_radius, 0, 0)
                                        * palm->getPose() );
    forearm_palm->init(odeHandle, osgHandle, conf.draw_joints, 0.7);
    joints.push_back(forearm_palm);


    //-------------AngularMotor for BallJoint---------------------------------------

    Axis axis1 = Axis(1,0,0)*palm->getPose();
    Axis axis3 = Axis(0,0,1)*palm->getPose();

    palm_motor_joint = new AngularMotor3AxisEuler(odeHandle, (BallJoint*) forearm_palm,
                                                  axis1, axis3, conf.power);
    if (conf.fix_palm_joint){
      ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop, 0);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop, 0);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop2, 0);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop2, 0);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop3, 0);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop3, 0);
    } else {
      ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop, -M_PI/4);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop, M_PI/2);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop2, -M_PI/4);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop2, M_PI/4);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop3, 0);
      ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop3, 0);
    }
    frictionmotors.push_back(palm_motor_joint);


    //------------------------thumb ----------------------------------------------

    Primitive *thumb_b,*thumb_t;//,*thumb_c;

    thumb_b = new Capsule(0.2,0.7);
    thumb_b -> init ( odeHandle , 0.1 , osgHandle ,
                      Primitive::Body|Primitive::Geom|Primitive::Draw);

    if (conf.initWithOpenHand){
      thumb_b ->setPose(osg::Matrix::rotate(M_PI/2, osg::Vec3(1, 0, 0),
                                            M_PI/2, osg::Vec3(0, 1, 0),
                                            0.0, osg::Vec3(0, 0, 1))
                        * osg::Matrix::translate(-0.8,-1.0, 0) * (palm->getPose()));
    } else {
      // todo: rotation not correct
      thumb_b ->setPose(osg::Matrix::rotate(M_PI/6, osg::Vec3(1, 0, 0),
                                            0.0, osg::Vec3(0, 1, 0),
                                            -2*M_PI,osg::Vec3(0, 0, 1))
                        * osg::Matrix::translate(-0.8,-1.0, 0.3) *(palm->getPose()));
    }

    objects.push_back(thumb_b);


    if(conf.ir_sensor_used && conf.irs_at_fingerbottom){
      IRSensor* sensor_thumb_b = new IRSensor();
      ir_sensors.push_back(sensor_thumb_b);
      irSensorBank.registerSensor(sensor_thumb_b, thumb_b,//objects[2],
        osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),M_PI/2,
                            osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1))
        * osg::Matrix::translate(0, 0.2, 0.15), conf.irRange, conf.ray_draw_mode);
    }


    thumb_t = new Capsule(0.2,0.5);
    thumb_t -> init ( odeHandle , 0.1 , osgHandle ,
                      Primitive::Body|Primitive::Geom|Primitive::Draw);
    thumb_t ->setPose(osg::Matrix::translate(0, 0, 0.7)*(thumb_b->getPose()) );
    objects.push_back(thumb_t);

    if(conf.ir_sensor_used && (conf.irs_at_fingertop || conf.irs_at_fingercenter)){ // fingercenter to have always 5 IR sensors
      IRSensor* sensor_thumb_t = new IRSensor();
      ir_sensors.push_back(sensor_thumb_t);
      irSensorBank.registerSensor(sensor_thumb_t, thumb_t,//objects[3],
                                  osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),
                                                      M_PI/2, osg::Vec3(0, 1, 0),
                                                      0.0,osg::Vec3(0, 0, 1)) *
                                  osg::Matrix::translate((0), 0.2, 0),
                                  conf.irRange, conf.ray_draw_mode);
    }

    if(conf.ir_sensor_used && (conf.irs_at_fingertip || conf.irs_at_fingercenter)){
      IRSensor* sensor_thumb_t = new IRSensor();
      ir_sensors.push_back(sensor_thumb_t);
      irSensorBank.registerSensor(sensor_thumb_t, thumb_t,//objects[3],
                                  osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),
                                                      M_PI/2, osg::Vec3(0, 1, 0),
                                                      0.0,osg::Vec3(0, 0, 1)) *
                                  osg::Matrix::translate((0), 0.2, 0.25),
                                  conf.irRange, conf.ray_draw_mode);
      /*
       * added same sensor as above to have as many IR's as motors (to motors in thumb)
       * TODO: remove (since it is only for testing InvertNChannelControllerHebbH)
       */
      IRSensor* sensor_thumb_t2 = new IRSensor();
      ir_sensors.push_back(sensor_thumb_t2);
      irSensorBank.registerSensor(sensor_thumb_t2, thumb_t,//objects[3],
                                  osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),
                                                      M_PI/2, osg::Vec3(0, 1, 0),
                                                      0.0,osg::Vec3(0, 0, 1)) *
                                  osg::Matrix::translate((0), 0.2, 0.25),
                                  conf.irRange, conf.ray_draw_mode);

    }


    //-------ball joint between palm and thumb----------------------------------------

    Joint* palm_thumb = new BallJoint(palm, thumb_b,
                                      thumb_b->getPosition()
                                      +(thumb_b->getPosition()-thumb_t->getPosition())/2);
    palm_thumb->init(odeHandle, osgHandle, conf.draw_joints, 0.3);
    joints.push_back(palm_thumb);

    //-------hinge joint for thumb-----------------------------------------------------
    thumb_bt = new HingeJoint(thumb_b, thumb_t,
                              (thumb_b->getPosition()+thumb_t->getPosition())/2,
                              Axis(0, 0, 1)*palm->getPose());
    thumb_bt->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    thumb_bt ->setParam(dParamLoStop,  -conf.fingerJointBendAngle);
    thumb_bt ->setParam(dParamHiStop,  0);
    thumb_bt ->setParam (dParamBounce, 0.9 );
    joints.push_back(thumb_bt);

    /*
    HingeServo* servo_bt =  new HingeServo((HingeJoint*)thumb_bt, -M_PI*3/8,
                                           0, conf.servo_motor_Power);
    servos.push_back(servo_bt);
    */

    //------------AngularMotor for BallJoint ---------------

    thumb_motor_joint = new AngularMotor3AxisEuler(odeHandle, (BallJoint*)palm_thumb,
                                                   axis1, axis3, conf.power);
    ((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop, -M_PI/360);
    ((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop,  0.8 * M_PI);

    ((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop2, 0);
    ((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop2, 0);
    ((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop3, -M_PI/8);
    ((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop3, 0);
    frictionmotors.push_back(thumb_motor_joint);


    //-----------index finger--------------------------------------------------

    Primitive *index_b, *index_c, *index_t;

    index_b = new Capsule(0.2,0.6);
    index_b -> init ( odeHandle , 0.1 , osgHandle ,
                      Primitive::Body|Primitive::Geom|Primitive::Draw);

    if (conf.initWithOpenHand){
    index_b ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0)
                      *osg::Matrix::translate((1.0), (-0.6), (-0.05)) *(palm->getPose()));
    } else {
      index_b ->setPose(osg::Matrix::rotate(M_PI/6, 0, 1, 0)
                        *osg::Matrix::translate((0.8), (-0.6), (0.3)) *(palm->getPose()));
    }

    objects.push_back(index_b);



    if(conf.ir_sensor_used && conf.irs_at_fingerbottom){
      IRSensor* sensor_index_b = new IRSensor();
      ir_sensors.push_back(sensor_index_b);
      irSensorBank.registerSensor(sensor_index_b, index_b,//objects[5],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0),
                                  conf.irRange, conf.ray_draw_mode);
    }



    index_c = new Capsule(0.2,0.6);
    index_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                      | Primitive::Geom | Primitive::Draw);
    index_c ->setPose(osg::Matrix::translate((0), (0), (0.59))*(index_b->getPose()));
    objects.push_back(index_c);

    if(conf.ir_sensor_used && conf.irs_at_fingercenter)
      {
        IRSensor* sensor_index_c = new IRSensor();
        ir_sensors.push_back(sensor_index_c);
        irSensorBank.registerSensor(sensor_index_c, index_c,//objects[6],
                                    osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                    osg::Matrix::translate((-0.2), 0, 0), conf.irRange,
                                    conf.ray_draw_mode);
      }

    index_t = new Capsule(0.2,0.5);
    index_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                      | Primitive::Geom | Primitive::Draw);
    index_t ->setPose(osg::Matrix::translate((0), (0), (0.59))*(index_c->getPose()));
    objects.push_back(index_t);

    if(conf.ir_sensor_used && conf.irs_at_fingertop){
      IRSensor* sensor_index_t = new IRSensor();
      ir_sensors.push_back(sensor_index_t);
      irSensorBank.registerSensor(sensor_index_t, index_t,//objects[7],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0), conf.irRange,
                                  conf.ray_draw_mode);
    }
    if(conf.ir_sensor_used && conf.irs_at_fingertip){
      IRSensor* sensor_index_t = new IRSensor();
      ir_sensors.push_back(sensor_index_t);
      irSensorBank.registerSensor(sensor_index_t, index_t,//objects[7],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0.3), conf.irRange,
                                  conf.ray_draw_mode);
    }
    if (conf.showFingernails){
      Primitive* index_nail = new Cylinder(0.19,0.0001);
      Primitive* fix_index_nail= new Transform(objects[7],index_nail,
                                               osg::Matrix::translate(-0.35, 0,0.205)
                                               *osg::Matrix::rotate(M_PI/2,0,1,0));
      fix_index_nail -> init (odeHandle , 0 , osgHandle);
    }

    //----------------index finger joints----------------------------------------------

    palm_index = new HingeJoint(palm, index_b, index_b->getPosition()
                                -(index_c->getPosition()-index_b->getPosition())/1.8,
                                Axis(0, 1, 0)*palm->getPose());
    palm_index ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );

    if (conf.initWithOpenHand){
      palm_index ->setParam(dParamLoStop,  0);
      palm_index ->setParam(dParamHiStop,  conf.fingerJointBendAngle);
    } else {
      palm_index ->setParam(dParamLoStop,  -M_PI/3);
      palm_index ->setParam(dParamHiStop,  (conf.fingerJointBendAngle-M_PI/3) );
    }
    palm_index ->setParam (dParamBounce, 0.9 );
    joints.push_back(palm_index);

    /*
    HingeServo* servo_palm_index =  new HingeServo((HingeJoint*)palm_index, conf.jointLimit1,
                                                   conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_palm_index);
    */

    index_bc = new HingeJoint(index_b, index_c,
                              (index_b->getPosition()+index_c->getPosition())/2,
                              Axis(0, 1, 0)*palm->getPose());
    index_bc ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    index_bc ->setParam(dParamLoStop, 0);
    index_bc ->setParam(dParamHiStop, conf.fingerJointBendAngle);
    index_bc ->setParam (dParamBounce, 0.9 );
    joints.push_back(index_bc);
    /*
    HingeServo* servo_index_bc =  new HingeServo((HingeJoint*)index_bc, conf.jointLimit1,
                                                 conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_index_bc);
    */

    index_ct = new HingeJoint(index_c, index_t,
                              ((index_c->getPosition()+index_t->getPosition())/2),
                              Axis(0, 1, 0)*palm->getPose());
    index_ct ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    index_ct ->setParam(dParamLoStop, 0);
    index_ct ->setParam(dParamHiStop,  conf.fingerJointBendAngle);
    index_ct ->setParam (dParamBounce, 0.9 );
    joints.push_back(index_ct);
    /*
    HingeServo* servo_index_ct =  new HingeServo((HingeJoint*)index_ct, conf.jointLimit1,
                                                 conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_index_ct);
    */

    //---------middle finger----------------------------------------------------------

    Primitive *middle_b,*middle_c,*middle_t;


    middle_b = new Capsule(0.2,0.6);
    middle_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                       | Primitive::Geom | Primitive::Draw);
    if (conf.initWithOpenHand) {
      middle_b->setPose(osg::Matrix::translate((0), (0.45), (0.3))*(index_b->getPose()));
    } else {
      middle_b->setPose(osg::Matrix::translate((0.20), (0.45), (0.20))*(index_b->getPose()));
    }

    objects.push_back(middle_b);
    if(conf.ir_sensor_used && conf.irs_at_fingerbottom){
        IRSensor* sensor_middle_b = new IRSensor();
        ir_sensors.push_back(sensor_middle_b);
        irSensorBank.registerSensor(sensor_middle_b, middle_b,//objects[8],
                                    osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                    osg::Matrix::translate((-0.2), 0, 0),  conf.irRange,
                                    conf.ray_draw_mode );
      }

    middle_c = new Capsule(0.2,0.7);
    middle_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                       | Primitive::Geom | Primitive::Draw);
    middle_c->setPose(osg::Matrix::translate((0), (0), (0.69))*(middle_b->getPose()));
    objects.push_back(middle_c);

    if(conf.ir_sensor_used && conf.irs_at_fingercenter) {
        IRSensor* sensor_middle_c = new IRSensor();
        ir_sensors.push_back(sensor_middle_c);
        irSensorBank.registerSensor(sensor_middle_c, middle_c,//objects[9],
                                    osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                    osg::Matrix::translate((-0.2), 0, 0), conf.irRange,
                                    conf.ray_draw_mode);
      }

    middle_t = new Capsule(0.2,0.7);
    middle_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                       | Primitive::Geom | Primitive::Draw);
    middle_t->setPose(osg::Matrix::translate((0), (0), (0.69))*(middle_c->getPose()));
    objects.push_back(middle_t);

    if(conf.ir_sensor_used && conf.irs_at_fingertop) {
      IRSensor* sensor_middle_t = new IRSensor();
      ir_sensors.push_back(sensor_middle_t);
      irSensorBank.registerSensor(sensor_middle_t, middle_t,//objects[10],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0),  conf.irRange,
                                  conf.ray_draw_mode);
    }
    if(conf.ir_sensor_used && conf.irs_at_fingertip) {
      IRSensor* sensor_middle_t = new IRSensor();
      ir_sensors.push_back(sensor_middle_t);
      irSensorBank.registerSensor(sensor_middle_t, middle_t,//objects[10],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0.35),  conf.irRange,
                                  conf.ray_draw_mode);
    }
    if (conf.showFingernails){
      Primitive* middle_nail = new Cylinder(0.19,0.0001);
      Primitive* fix_middle_nail= new Transform(objects[10],middle_nail,
                                                osg::Matrix::translate(-0.4, 0,0.205)
                                                *osg::Matrix::rotate(M_PI/2,0,1,0));
      fix_middle_nail -> init (odeHandle , 0 , osgHandle);
    }


    //-------------------------middle finger joints-------------------------------------

    palm_middle = new HingeJoint(palm, middle_b,middle_b->getPosition()
                                 -(middle_c->getPosition()-middle_b->getPosition())/1.8,
                                 Axis(0, 1, 0)*palm->getPose());
    palm_middle ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    if (conf.initWithOpenHand){
      palm_middle->setParam(dParamLoStop, 0);
      palm_middle->setParam(dParamHiStop, conf.fingerJointBendAngle);
    } else {
      palm_middle ->setParam(dParamLoStop,  -M_PI/3);
      palm_middle ->setParam(dParamHiStop,  (conf.fingerJointBendAngle-M_PI/3));
    }

    joints.push_back(palm_middle);
    /*
    HingeServo* servo_palm_middle =  new HingeServo((HingeJoint*)palm_middle, conf.jointLimit1,
                                                    conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_palm_middle);
    */

    middle_bc = new HingeJoint(middle_b, middle_c,
                               (middle_b->getPosition()+middle_c->getPosition())/2,
                               Axis(0, 1, 0)*palm->getPose());

    middle_bc ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    middle_bc->setParam(dParamLoStop, 0);
    middle_bc->setParam(dParamHiStop, conf.fingerJointBendAngle);
    joints.push_back(middle_bc);
    /*
    HingeServo* servo_middle_bc =  new HingeServo((HingeJoint*)middle_bc, conf.jointLimit1,
                                                  conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_middle_bc);
    */
    middle_ct = new HingeJoint(middle_c, middle_t,
                               (middle_c->getPosition()+middle_t->getPosition())/2,
                               Axis(0, 1, 0)*palm->getPose());
    middle_ct ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    middle_ct->setParam(dParamLoStop, 0);
    middle_ct->setParam(dParamHiStop, conf.fingerJointBendAngle);
    joints.push_back(middle_ct);
    /*
    HingeServo* servo_middle_ct =  new HingeServo((HingeJoint*)middle_ct, conf.jointLimit1,
                                                  conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_middle_ct);
    */


    //---------------------------ring finger-------------------------------------------
    Primitive *ring_b, *ring_c, *ring_t;

    ring_b = new Capsule(0.2,0.6);
    ring_b-> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                    | Primitive::Geom | Primitive::Draw);
    if (conf.initWithOpenHand){
      ring_b->setPose(osg::Matrix::translate((0), (0.9), (0.3))*(index_b->getPose()));
    } else {
      ring_b->setPose(osg::Matrix::translate((0.20), (0.9), (0.20))*(index_b->getPose()));
    }

    objects.push_back(ring_b);

    if(conf.ir_sensor_used && conf.irs_at_fingerbottom) {
      IRSensor* sensor_ring_b = new IRSensor();
      ir_sensors.push_back(sensor_ring_b);
      irSensorBank.registerSensor(sensor_ring_b, ring_b,//objects[11],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0),  conf.irRange,
                                  conf.ray_draw_mode );
    }

    ring_c = new Capsule(0.2,0.6);
    ring_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                     | Primitive::Geom | Primitive::Draw);
    ring_c->setPose(osg::Matrix::translate((0), (0), (0.59))*(ring_b->getPose()));
    objects.push_back(ring_c);

    if(conf.ir_sensor_used && conf.irs_at_fingercenter) {
      IRSensor* sensor_ring_c = new IRSensor();
      ir_sensors.push_back(sensor_ring_c);
      irSensorBank.registerSensor(sensor_ring_c, ring_c,//objects[12],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0),  conf.irRange,
                                  conf.ray_draw_mode);
    }

    ring_t = new Capsule(0.2,0.5);
    ring_t -> init ( odeHandle, 0.1 , osgHandle , Primitive::Body
                     | Primitive::Geom | Primitive::Draw);
    ring_t->setPose(osg::Matrix::translate((0), (0), (0.59))*(ring_c->getPose()));
    objects.push_back(ring_t);

    if(conf.ir_sensor_used && conf.irs_at_fingertop){
      IRSensor* sensor_ring_t = new IRSensor();
      ir_sensors.push_back(sensor_ring_t);
      irSensorBank.registerSensor(sensor_ring_t, ring_t,//objects[13],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0),  conf.irRange,
                                  conf.ray_draw_mode);
    }
    if(conf.ir_sensor_used && conf.irs_at_fingertip){
      IRSensor* sensor_ring_t = new IRSensor();
      ir_sensors.push_back(sensor_ring_t);
      irSensorBank.registerSensor(sensor_ring_t, ring_t,//objects[13],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0.25),  conf.irRange,
                                  conf.ray_draw_mode);
    }
    if (conf.showFingernails){
      Primitive* ring_nail = new Cylinder(0.19,0.0001);
      Primitive* fix_ring_nail= new Transform(objects[13],ring_nail,
                                              osg::Matrix::translate(-0.37, 0,0.205)
                                              *osg::Matrix::rotate(M_PI/2,0,1,0));
      fix_ring_nail -> init (odeHandle , 0 , osgHandle);
    }


    //-----------ring finger joints----------------------------------------------------


    palm_ring = new HingeJoint(palm, ring_b, ring_b->getPosition()
                               -(ring_c->getPosition()-ring_b->getPosition())/1.8,
                               Axis(0, 1, 0)*palm->getPose());
    palm_ring->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    if (conf.initWithOpenHand){
      palm_ring->setParam(dParamLoStop,  0);
      palm_ring->setParam(dParamHiStop,  conf.fingerJointBendAngle);
    } else {
      palm_ring ->setParam(dParamLoStop,  -M_PI/3);
      palm_ring ->setParam(dParamHiStop,  (conf.fingerJointBendAngle - M_PI/3) );
    }

    joints.push_back(palm_ring);
    /*
    HingeServo* servo_palm_ring =  new HingeServo((HingeJoint*)palm_ring, conf.jointLimit1,
                                                  conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_palm_ring);
    */
    ring_bc = new HingeJoint(ring_b, ring_c, (ring_b->getPosition()+ring_c->getPosition())/2,
                             Axis(0, 1, 0)*palm->getPose());
    ring_bc ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    ring_bc->setParam(dParamLoStop, 0);
    ring_bc->setParam(dParamHiStop, conf.fingerJointBendAngle);
    joints.push_back(ring_bc);
    /*
    HingeServo* servo_ring_bc =  new HingeServo((HingeJoint*)ring_bc, conf.jointLimit1,
                                                conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_ring_bc);
    */

    ring_ct = new HingeJoint(ring_c, ring_t, (ring_c->getPosition()+ring_t->getPosition())/2,
                             Axis(0, 1, 0)*palm->getPose());
    ring_ct ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    ring_ct->setParam(dParamLoStop, 0);
    ring_ct->setParam(dParamHiStop, conf.fingerJointBendAngle);
    joints.push_back(ring_ct);
    /*
    HingeServo* servo_ring_ct =  new HingeServo((HingeJoint*)ring_ct, conf.jointLimit1,
                                                conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_ring_ct);
    */


    //-----------little finger--------------------------------------------------------

    Primitive *little_b, *little_c, *little_t;

    little_b = new Capsule(0.2,0.6);
    little_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                       | Primitive::Geom | Primitive::Draw);
    little_b->setPose(osg::Matrix::translate((0), (1.35), (0))*(index_b->getPose()));
    objects.push_back(little_b);

    if(conf.ir_sensor_used && conf.irs_at_fingerbottom) {
        IRSensor* sensor_little_b = new IRSensor();
        ir_sensors.push_back(sensor_little_b);
        irSensorBank.registerSensor(sensor_little_b, little_b,//objects[14],
                                    osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                    osg::Matrix::translate((-0.2), 0, 0),   conf.irRange,
                                    conf.ray_draw_mode);
      }

    little_c = new Capsule(0.2,0.6);
    little_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                       | Primitive::Geom | Primitive::Draw);

    little_c->setPose(osg::Matrix::translate((0), (0), (0.59))*(little_b->getPose()));
    objects.push_back(little_c);

    if(conf.ir_sensor_used && conf.irs_at_fingercenter)  {
        IRSensor* sensor_little_c = new IRSensor();
        ir_sensors.push_back(sensor_little_c);
        irSensorBank.registerSensor(sensor_little_c, little_c,//objects[15],
                                    osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                    osg::Matrix::translate((-0.2), 0, 0),   conf.irRange,
                                    conf.ray_draw_mode);
      }

    little_t = new Capsule(0.2,0.6);
    little_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body
                       | Primitive::Geom | Primitive::Draw);
    little_t->setPose(osg::Matrix::translate((0), (0), (0.59))*(little_c->getPose()));
    objects.push_back(little_t);

    if(conf.ir_sensor_used && conf.irs_at_fingertop) {
      IRSensor* sensor_little_t = new IRSensor();
      ir_sensors.push_back(sensor_little_t);
      irSensorBank.registerSensor(sensor_little_t, little_t,//objects[16],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0),   conf.irRange,
                                  conf.ray_draw_mode);
    }
    if(conf.ir_sensor_used && conf.irs_at_fingertip) {
      IRSensor* sensor_little_t = new IRSensor();
      ir_sensors.push_back(sensor_little_t);
      irSensorBank.registerSensor(sensor_little_t, little_t,//objects[16],
                                  osg::Matrix::rotate(-M_PI/2, 0, 1, 0)  *
                                  osg::Matrix::translate((-0.2), 0, 0.3),   conf.irRange,
                                  conf.ray_draw_mode);
    }
    if (conf.showFingernails){
      Primitive* little_nail = new Cylinder(0.19,0.0001);
      Primitive* fix_little_nail= new Transform(objects[16],little_nail,
                                                osg::Matrix::translate(-0.39, 0,0.205)
                                                *osg::Matrix::rotate(M_PI/2,0,1,0));
      fix_little_nail -> init (odeHandle , 0 , osgHandle);
    }


    //-----------------little finger joints-----------------------------------------


    palm_little = new HingeJoint(palm, little_b,
        little_b->getPosition()-(little_c->getPosition()-little_b->getPosition())/1.8,
        Axis(0, 1, 0)*palm->getPose());
    palm_little ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    if (conf.initWithOpenHand) {
      palm_little ->setParam(dParamLoStop, 0);
      palm_little ->setParam(dParamHiStop, conf.fingerJointBendAngle);
    } else {
      palm_little  ->setParam(dParamLoStop,  -M_PI/3);
      palm_little  ->setParam(dParamHiStop,  (conf.fingerJointBendAngle - M_PI/3) );
    }

    joints.push_back(palm_little);
    /*
    HingeServo* servo_palm_little =  new HingeServo((HingeJoint*)palm_little, conf.jointLimit1,
                                                    conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_palm_little);
    */
    little_bc = new HingeJoint(little_b, little_c,
                               (little_b->getPosition()+little_c->getPosition())/2,
                               Axis(0, 1, 0)*palm->getPose());
    little_bc ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    little_bc ->setParam(dParamLoStop, 0);
    little_bc ->setParam(dParamHiStop, conf.fingerJointBendAngle);
    joints.push_back(little_bc);
    /*
    HingeServo* servo_little_bc =  new HingeServo((HingeJoint*)little_bc, conf.jointLimit1,
                                                  conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_little_bc);
    */
    little_ct = new HingeJoint(little_c, little_t,
                               (little_c->getPosition()+little_t->getPosition())/2,
                               Axis(0, 1, 0)*palm->getPose());
    little_ct ->init(odeHandle, osgHandle, conf.draw_joints, 0.7 );
    little_ct ->setParam(dParamLoStop, 0);
    little_ct ->setParam(dParamHiStop, conf.fingerJointBendAngle);
    joints.push_back(little_ct);

    /*
    HingeServo* servo_little_ct =  new HingeServo((HingeJoint*)little_ct, conf.jointLimit1,
                                                  conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_little_ct);
    */




    // TDOD add following as conf option prostheses-like
    //-joints connecting index, middle, ring and little finger so that they move together
    /*
      Joint *index_middle, *middle_ring, *ring_little;


      index_middle = new HingeJoint(index_b, middle_b,
      Pos(1.4+conf.x, -0.375+conf.y, 0.5+conf.z), Axis(1, 0, 0));
      index_middle ->init(odeHandle, osgHandle, true, 0.2 );
      index_middle ->setParam(dParamLoStop,  M_PI/360);
      index_middle ->setParam(dParamHiStop, -M_PI/360);
      joints.push_back(index_middle);

      middle_ring = new HingeJoint(middle_b, ring_b,
      Pos(1.4+conf.x, 0.075+conf.y, 0.5+conf.z), Axis(1, 0, 0));
      middle_ring ->init(odeHandle, osgHandle, true, 0.2 );
      middle_ring ->setParam(dParamLoStop,  M_PI/360);
      middle_ring ->setParam(dParamHiStop, -M_PI/360);
      joints.push_back(middle_ring);

      ring_little = new HingeJoint(ring_b, little_b,
      Pos(1.4+conf.x, 0.525+conf.y, 0.5+conf.z), Axis(1, 0, 0));
      ring_little ->init(odeHandle, osgHandle, true, 0.2 );
      ring_little ->setParam(dParamLoStop,  M_PI/360);
      ring_little ->setParam(dParamHiStop, -M_PI/360);

      joints.push_back(ring_little);
    */
    created=true;
  }


  /** destroys robot
   */
  void Hand::destroy(){
    if (created){

      for (std::vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++)
        {
          if(*i) delete *i;
        }
      objects.clear();

      for (std::vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
        if(*i) delete *i;
      }
      joints.clear();

      for (std::vector<HingeServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
        if(*i) delete *i;
      }
      servos.clear();

      for (std::vector<AngularMotor*>::iterator i = frictionmotors.begin(); i!=                         frictionmotors.end(); i++){
        if(*i) delete *i;
      }
      frictionmotors.clear();

      irSensorBank.clear();

      dJointGroupDestroy (odeHandle.jointGroup);
      dSpaceDestroy(odeHandle.space);
      dWorldDestroy (odeHandle.world);
      //        dCloseODE();
    }
    created=false;
  }




  void Hand::notifyOnChange(const paramkey& key){
    //if(key == "jointLimit") conf.jointLimit1=val;
    //else
    if(key == "servo_motor_Power") {
      for (std::vector<HingeServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
        if(*i) (*i)->setPower(conf.servo_motor_Power);
      }
    }
    else if(key == "irRange") {
      for (unsigned int i=2; i< objects.size(); i++){
        //     for (std::vector<Primitive*>::iterator i = objects.begin()+2; i!= objects.end(); i++){
        irSensorBank.registerSensor(ir_sensors[i-2], objects[i],
                                    osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) *
                                    osg::Matrix::translate((0), 0.2, 0), conf.irRange,
                                    conf.ray_draw_mode);

      }
      irSensorBank.update();
    }
  }

}

