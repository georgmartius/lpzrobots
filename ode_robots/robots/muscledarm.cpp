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

#include <iostream>
#include <assert.h>

#include "mathutils.h"
#include "muscledarm.h"

using namespace std;

namespace lpzrobots{

  MuscledArm::MuscledArm(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         const MuscledArmConf& conf, const std::string& name):
    OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(conf){

    parentspace=odeHandle.space;
    factorMotors=0.1;
    damping=1; // seems to be wrong! (too high)

    addParameter("factorMotors", &factorMotors, 0,50);
    addParameterDef("factorSensors", &factorSensors, 8.0,0,50);
    addParameter("damping", &damping, 0,1 );
    addParameterDef("print", &print, 3, 0,10 );


    sensorno=0;
    if (conf.jointAngleSensors)
      sensorno+=2;
    if (conf.jointAngleRateSensors)
      sensorno+=2+4;
    if (conf.muscleLengthSensors)
      sensorno+=6;

    if (conf.jointActuator){
      motorno=2;
    }else{
      motorno=6;
    }

    print=0;

    this->osgHandle.color = Color(1,1,0);
    max_l=0;
    max_r=0;
    min_l=10;
    min_r=10;

    for (int i=0; i<NUMParts; i++){
      old_dist[i].x=0;
      old_dist[i].y=0;
      old_dist[i].z=0;
    }

    for (int i=0; i<6; i++){
      force_[i]=0;
    }

    base_width=SIDE;
    base_length=SIDE*1.7;
    upperArm_width=SIDE*0.2;
    upperArm_length=SIDE*3.0;
    lowerArm_width=SIDE*0.2;
    lowerArm_length=SIDE*4.0;
    mainMuscle_width=SIDE*0.2;
    mainMuscle_length=SIDE*2;
    smallMuscle_width=SIDE*0.1;
    smallMuscle_length=SIDE*0.5;

    joint_offset=0.01;

    created=false;
  };

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void MuscledArm::setMotorsIntern(const double* motors, int motornumber){
    if (!conf.jointActuator) {
      for (int i=SJ_mM1; i<=SJ_sM4; i++){
        // just adding force to slider joint
        //((SliderJoint*)joint[i])->addForce(factorMotors * motors[i-SJ_mM1]);

        //
        ((SliderJoint*)joint[i])->
          addForce1(factorMotors * (motors[i-SJ_mM1]- 8 * ((SliderJoint*)joint[i])->getPosition1()));
      }
    }else{
      for(int i=HJ_BuA; i<= HJ_uAlA; i++){ // two hinge joints
        //        ((HingeJoint*)joint[i])->addTorque(motors[i-HJ_BuA]);
        ((HingeJoint*)joint[i])->addForce1
          (M_PI/3 //range of joint is -M_PI/3 .. M_PI/3, and so is the result of getPosition1()
           * motors[i-HJ_BuA]
           - ((HingeJoint*)joint[i])->getPosition1());
      }
    }
  };


  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int MuscledArm::getSensorsIntern(sensor* sensors, int sensornumber){
    int written=0;
    if ((conf.jointAngleSensors) && ((written+1)<sensornumber) ){
      sensors[written]= 3/M_PI*((HingeJoint*)joint[HJ_BuA])->getPosition1();
      written++;
      sensors[written]= 3/M_PI*((HingeJoint*)joint[HJ_uAlA])->getPosition1();
      written++;
    }

    if ( (conf.jointAngleRateSensors) && ((written+1)<sensornumber) ) {
      sensors[written]= factorSensors * ((HingeJoint*)joint[HJ_BuA])->getPosition1Rate();
      written++;
      sensors[written]= factorSensors * ((HingeJoint*)joint[HJ_uAlA])->getPosition1Rate();
      written++;
      written--;
      // add 4 sensors with 0 value to have number sensor equal number motors when
      // using muscle actuation (needed by InvertNChannelController)
      for (int j=0; j<=4; j++, written++){
        sensors[written]=0.0;
      }
    }

    if ( (conf.muscleLengthSensors) && ((written+5)<sensornumber) ) {
      for (int j=SJ_mM1; j<=SJ_sM4; j++, written++){
        sensors[written]=factorSensors * ((SliderJoint*)joint[j])->getPosition1();
      }
    }
    //printf("written= %i \n",written);
    return written;
  };

  /** sets the pose of the vehicle
      @param pose desired 4x4 pose matrix
  */
  void MuscledArm::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the base
    // to set the arm on the ground when the z component of the position is 0
    // base_width/2 is added
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, base_width/2));
    create(p2);
  };


  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments)
      @return length of the list
  */
  int MuscledArm::getSegmentsPosition(std::vector<Position> &poslist){
    assert(created);
    for (int i=0; i<NUMParts; i++){
      poslist.push_back(Position(dBodyGetPosition(object[i]->getBody())));
    }
    return NUMParts;
  };



  void MuscledArm::doInternalStuff(GlobalData& globalData){
    OdeRobot::doInternalStuff(globalData);

    double k[6];
    k[0] = 0.5;        // spring constant between mainMuscle11 and mainMuscle12
    k[1] = 0.2;        // spring constant between mainMuscle21 and mainMuscle22
    k[2] = 0.5;        // spring constant between smallMuscle11 and smallMuscle12
    k[3] = 0.2;        // spring constant between smallMuscle21 and smallMuscle22
    k[4] = 0.5;        // spring constant between smallMuscle31 and smallMuscle32
    k[5] = 0.2;        // spring constant between smallMuscle41 and smallMuscle42

    //          damping=0.1;

    //          double force;
    //          for(int i=SJ_mM1; i<(SJ_sM4+1); i++){
    //            //calculating force
    //            force=((SliderJoint*)joint[i])->getPosition1()  * k[i];
    //            //damping
    //            force=force + damping * ((SliderJoint*)joint[i])->getPosition1Rate() ;
    //            ((SliderJoint*)joint[i])->addForce(0.1*force);
    //          }


    // add a spring force to keep the bodies together, otherwise they will
    // fly apart along the slider axis.

    //         //mainMuscle11  =  3
    //         //smallMuscle42 = 15
    const dReal *p1;
    const dReal *p2;
    for (int i=mainMuscle11; i<smallMuscle42; i+=2){
      p1 = dBodyGetPosition (object[i]->getBody());
      p2 = dBodyGetPosition (object[i+1]->getBody());

      Position dist;
      //distance between slider joint elements
      dist.x=p2[0]-p1[0];
      dist.y=p2[1]-p1[1];
      dist.z=p2[2]-p1[2];

      Position force;
      //calculating force
      force=dist*k[(int)(i*0.5)-1];

      //damping
      force=force + (dist - old_dist[i] )*damping;

      dBodyAddForce (object[i]->getBody(), force.x, force.y, force.z);
      dBodyAddForce (object[i+1]->getBody(), -force.x, -force.y, -force.z);
      old_dist[i]=dist;
    }

  }

  void MuscledArm::mycallback(void *data, dGeomID o1, dGeomID o2){
    MuscledArm* me = (MuscledArm*)data;

    if (// collision between fixed body and upper arm
        ((o1 == me->object[base]->getGeom()) && (o2 == me->object[upperArm]->getGeom()))
        || ((o2 == me->object[base]->getGeom()) && (o1 == me->object[upperArm]->getGeom()))
        // collision between upper arm and lower arm
        || ((o1 == me->object[upperArm]->getGeom()) && (o2 == me->object[lowerArm]->getGeom()))
        || ((o2 == me->object[upperArm]->getGeom()) && (o1 == me->object[lowerArm]->getGeom()))
        // collision between fixed body and lower arm
        || ((o1 == me->object[base]->getGeom()) && (o2 == me->object[lowerArm]->getGeom()))
        || ((o2 == me->object[base]->getGeom()) && (o1 == me->object[lowerArm]->getGeom()))
        ){

      int i,n;
      const int N = 10;
      dContact contact[N];

      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++) {
        contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
        contact[i].surface.mu = 0.01;
        contact[i].surface.soft_erp = 1;
        contact[i].surface.soft_cfm = 0.00001;
        dJointID c = dJointCreateContact( me->odeHandle.world, me->odeHandle.jointGroup, &contact[i]);
        dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
      }
    }

  }

  /** creates arm at desired position
  */
  void MuscledArm::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }
    // create vehicle space and add it to parentspace
    odeHandle.createNewSimpleSpace(parentspace,false);

    // create base
    object[base] = new Box(base_width, base_width, base_length);
    object[base] -> init(odeHandle, MASS, osgHandle,Primitive::Geom  | Primitive::Draw);
    //    if(conf.strained){

    // place base
    object[base] -> setPose(osg::Matrix::rotate(M_PI/2, 1, 0, 0)
                            * pose);
    // create and place upper arm
    object[upperArm] = new Box(upperArm_width, upperArm_width, upperArm_length);
    this->osgHandle.color = Color(1, 0, 0, 1.0f);
    object[upperArm] -> init(odeHandle, MASS, osgHandle);
    object[upperArm] -> setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate
                                (-(base_width/2+upperArm_length/2)-joint_offset,0,0) * pose);
    // create and place lower arm
    object[lowerArm] = new Box(lowerArm_width, lowerArm_width, lowerArm_length);
    this->osgHandle.color = Color(0,1,0);
    object[lowerArm] -> init(odeHandle, MASS, osgHandle);
    object[lowerArm] -> setPose(osg::Matrix::rotate(M_PI/2, 1, 0, 0) * osg::Matrix::translate
                                (-(base_width/2+upperArm_length+lowerArm_width/2+2*joint_offset),
                                 lowerArm_length/4,0) * pose);


    // create and place sphere at tip of lower arm
    // sphere as transform object
//     osg::Matrix ps;
//     ps.makeIdentity();
//     Primitive* o1 = new Sphere(lowerArm_width);
//     // move to end of arm in local coordinates
//     object[hand] = new Transform(object[lowerArm], o1,
//                                  osg::Matrix::translate(0, 0,-lowerArm_length*0.5) * ps);
//     object[hand]->init(odeHandle, /*mass*/0, osgHandle, Primitive::Geom  | Primitive::Draw);

    // hand with fixed joint connected to lowerArm to allow tracing
    // when sphere is transform object (-> only sphere) tracing does not work
    // => do this better
    object[hand] = new Sphere(lowerArm_width);
    object[hand] -> init(odeHandle, /*mass*/0.01, osgHandle);
    object[hand] -> setPose(osg::Matrix::rotate(M_PI/2, 1, 0, 0) * osg::Matrix::translate
                                (-(base_width/2+upperArm_length+lowerArm_width/2+2*joint_offset),
                                 lowerArm_length/4 + lowerArm_length/2,0) * pose);
    joint[FJ_lAH] = new FixedJoint(object[lowerArm], object[hand]);
    joint[FJ_lAH]->init(odeHandle, osgHandle, false);


    osg::Vec3 pos;
    // hinge joint between upper arm and fixed body
    pos=object[base]->getPosition();
    joint[HJ_BuA] = new HingeJoint(object[base], object[upperArm],
                                   osg::Vec3(pos[0]-base_width/2, pos[1], pos[2]),
                                   osg::Vec3(0, 0, 1));
    joint[HJ_BuA]->init(odeHandle, osgHandle, true);
    // set stops to make sure arm stays in useful range
    joint[HJ_BuA]->setParam(dParamLoStop,-M_PI/3);
    joint[HJ_BuA]->setParam(dParamHiStop, M_PI/3);

    // hinge joint between upperArm and lowerArm
    pos=object[upperArm]->getPosition();
    joint[HJ_uAlA] = new HingeJoint(object[upperArm], object[lowerArm],
                                    osg::Vec3(pos[0]-upperArm_length/2-joint_offset,
                                              pos[1], pos[2]),
                                    osg::Vec3(0, 0, 1));
    joint[HJ_uAlA]->init(odeHandle, osgHandle, true);
    // set stops to make sure arm stays in useful range
    joint[HJ_uAlA]->setParam(dParamLoStop,-M_PI/3);
    joint[HJ_uAlA]->setParam(dParamHiStop, M_PI/3);



    //       if (conf.includeMuscles) {

    // create and place boxes for mainMuscles
    for (int i= mainMuscle11; i<smallMuscle11; i++){
      object[i] = new Box(mainMuscle_width, mainMuscle_width, mainMuscle_length);
      //object[i] = new Capsule(mainMuscle_width/2,mainMuscle_length);
      object[i] -> init(odeHandle, MASS, osgHandle);
      if (i==mainMuscle11) this->osgHandle.color = Color(0.4,0.4,0);
      if (i==mainMuscle12) this->osgHandle.color = Color(0,1,1);
      if (i==mainMuscle21) this->osgHandle.color = Color(1,1,0);
    }
    /* left main Muscle */
    /**************************************/
    pos=object[upperArm]->getPosition();
    object[mainMuscle11] -> setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate
                                    (// moved towards base, aligned with end of upperArm
                                     pos[0]+(upperArm_length-mainMuscle_length)/2,
                                     // moved to left of upper arm, aligned with end of base
                                     pos[1]-(base_length/2-mainMuscle_width/2),
                                     0)  // height is ok
                                    * pose);
    object[mainMuscle12] -> setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate
                                    (// moved away from base, aligned with end of upperArm
                                     pos[0]-(upperArm_length-mainMuscle_length)/2,
                                     // moved to left of upper arm, aligned with end of base
                                     pos[1]-(base_length/2-mainMuscle_width/2),
                                     0)  // height is ok
                                    * pose);
    /* right main Muscle */
    /**************************************/
    object[mainMuscle21] -> setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate
                                    (// moved towards base, aligned with end of upperArm
                                     pos[0]+(upperArm_length-mainMuscle_length)/2,
                                     // moved to right of upper arm, aligned with end of base
                                     pos[1]+(base_length/2-mainMuscle_width/2),
                                     0)  // height is ok
                                    * pose);
    object[mainMuscle22] -> setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate
                                    (// moved away from base, aligned with end of upper
                                     pos[0]-(upperArm_length-mainMuscle_length)/2,
                                     // moved to right of upper arm, aligned with end of base
                                     pos[1]+(base_length/2-mainMuscle_width/2),
                                     0)  // height is ok
                                    * pose);
    /* joints for left main Muscle */
    /**************************************/
    // hinge joint between mainMuscle11 and fixed body */
    pos=object[mainMuscle11]->getPosition();
    joint[HJ_BmM11] = new HingeJoint(object[base], object[mainMuscle11],
                                     osg::Vec3(pos[0]+mainMuscle_length/2+joint_offset,
                                               pos[1], pos[2]), osg::Vec3(0, 0, 1));
    joint[HJ_BmM11]->init(odeHandle, osgHandle, true);

    // hinge joint between mainMuscle12 and lowerArm */
    pos=object[mainMuscle12]->getPosition();
    joint[HJ_lAmM12] = new HingeJoint(object[lowerArm], object[mainMuscle12],
                                      osg::Vec3(pos[0]-mainMuscle_length/2-joint_offset,
                                                pos[1], pos[2]), osg::Vec3(0, 0, 1));
    joint[HJ_lAmM12]->init(odeHandle, osgHandle, true);
    // slider joint between mainMuscle11 and mainMuscle12
    joint[SJ_mM1] = new SliderJoint(object[mainMuscle11], object[mainMuscle12],
                                    /*anchor (not used)*/osg::Vec3(0, 0, 0),
                                    osg::Vec3(1, 0, 0));
    joint[SJ_mM1] -> init(odeHandle, osgHandle, /*withVisual*/ false);//true);


    /* joints for right main Muscle */
    /**************************************/
    // hinge joint between mainMuscle21 and fixed body */
    pos=object[mainMuscle21]->getPosition();
    joint[HJ_BmM21] = new HingeJoint(object[base], object[mainMuscle21],
                                     osg::Vec3(pos[0]+mainMuscle_length/2+joint_offset,
                                               pos[1], pos[2]), osg::Vec3(0, 0, 1));
    joint[HJ_BmM21]->init(odeHandle, osgHandle, true);

    // hinge joint between mainMuscle22 and lowerArm */
    pos=object[mainMuscle22]->getPosition();
    joint[HJ_lAmM22] = new HingeJoint(object[lowerArm], object[mainMuscle22],
                                      osg::Vec3(pos[0]-mainMuscle_length/2-joint_offset,
                                                pos[1], pos[2]), osg::Vec3(0, 0, 1));
    joint[HJ_lAmM22]->init(odeHandle, osgHandle, true);
    // slider joint between mainMuscle21 and mainMuscle22
    joint[SJ_mM2] = new SliderJoint(object[mainMuscle21], object[mainMuscle22],
                                    /*anchor (not used)*/osg::Vec3(0, 0, 0),
                                    osg::Vec3(1, 0, 0));
    joint[SJ_mM2] -> init(odeHandle, osgHandle, /*withVisual*/ false);//true);



    // create and place boxes for smallMuscles
    /*****************************************************************/
    for (int i= smallMuscle11; i<hand; i++){
      object[i] = new Box(smallMuscle_width, smallMuscle_width, smallMuscle_length);
      object[i] -> init(odeHandle, MASS, osgHandle);
    }
    /* lower left small Muscle */
    /**************************************/
    pos=object[mainMuscle11]->getPosition();
    object[smallMuscle11] -> setPose(osg::Matrix::rotate(M_PI*0.5, -1,-1, 0)* osg::Matrix::translate
                                     // move center of this box to lower end of mainMuscle11
                                     (pos[0]+mainMuscle_length/2
                                      // move box away from base to align lower edges of
                                      // mainMuscle11 and this muscle
                                      -smallMuscle_length/2,
                                      -smallMuscle_length,  // moved to left
                                      0) // height is ok
                                     * pose);
    pos=object[smallMuscle11]->getPosition();
    //object[smallMuscle12] -> setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, -1)* osg::Matrix::translate
    object[smallMuscle12] -> setPose(osg::Matrix::rotate(M_PI*0.5, -1, -1, 0)* osg::Matrix::translate
                                     (//calculate upper shift using sideward offset from smallMuscle11
                                      //(to align smallMuscle11 and this muscle)
                                      pos[0] -tan(M_PI/4)*(smallMuscle_length/2),
                                      -smallMuscle_length/2, // moved to left
                                      0) * pose); // heigth is ok
    /* lower right small Muscle */
    /**************************************/
    pos=object[smallMuscle11]->getPosition();
    object[smallMuscle21] -> setPose(osg::Matrix::rotate(M_PI*0.5, -1, 1, 0)* osg::Matrix::translate
                                     // place as smallMuscle11, accecpt that it is on the right side
                                     // of the upperArm (-> -pos[1])
                                     (pos[0], -pos[1], 0) * pose); //height is ok
    pos=object[smallMuscle12]->getPosition();
    object[smallMuscle22] -> setPose(osg::Matrix::rotate(M_PI*0.5, -1, 1, 0) * osg::Matrix::translate
                                     // place as smallMuscle12, accecpt that it is on the right side
                                     // of the upperArm (-> -pos[1])
                                     (pos[0], -pos[1], 0)* pose);  // height is ok
    /* upper left small Muscle */
    /**************************************/
    pos=object[mainMuscle12]->getPosition();
    object[smallMuscle31] -> setPose(osg::Matrix::rotate(M_PI*0.5, -1, 1, 0) * osg::Matrix::translate
                                     // move center of this box to lower end of mainMuscle12
                                     (pos[0]-mainMuscle_length/2
                                      // move box in direction of base to align upper edges of
                                      // mainMuscle12 and this muscle
                                      +smallMuscle_length/2,
                                      -smallMuscle_length,  // move to left
                                      0) // height is ok
                                     * pose);
    pos=object[smallMuscle31]->getPosition();
    object[smallMuscle32] -> setPose(osg::Matrix::rotate(M_PI*0.5, -1, 1, 0)* osg::Matrix::translate
                                     //calculate shift toweards base using sideward offset from
                                     //smallMuscle31 (to align smallMuscle11 and this muscle)
                                     (pos[0]+tan(M_PI/4)*(smallMuscle_length/2),
                                      pos[1]+smallMuscle_length/2, // move to left
                                      0)
                                     * pose);  // height is ok

    /* upper right small Muscle */
    /**************************************/
    pos=object[smallMuscle31]->getPosition();
    object[smallMuscle41] -> setPose(osg::Matrix::rotate(M_PI*0.5, -1, -1, 0) * osg::Matrix::translate
                                     // place as smallMuscle31, accecpt that it is on the right side
                                     // of the upperArm (-> -pos[1])
                                     (pos[0], -pos[1],
                                      0)* pose); //height is ok
    pos=object[smallMuscle32]->getPosition();
    object[smallMuscle42] -> setPose(osg::Matrix::rotate(M_PI*0.5, -1, -1, 0)* osg::Matrix::translate
                                     // place as smallMuscle32, accecpt that it is on the right side
                                     // of the upperArm (-> -pos[1])
                                     (pos[0], -pos[1],
                                      0) * pose); //height is ok

    /* joints for lower left small Muscle */
    /**************************************/
    // hinge joint between base and smallMuscle11 */
    joint[HJ_BsM11] = new HingeJoint(object[base], object[smallMuscle11],
                                     // same as HJ between base and mainMuscle11
                                     joint[HJ_BmM11]->getAnchor(),
                                     ((OneAxisJoint*)joint[HJ_BmM11])->getAxis1());
    joint[HJ_BsM11]->init(odeHandle, osgHandle, true);

    // hinge joint between upperArm and smallMuscle12 */
    pos=joint[HJ_BuA]->getAnchor();
    joint[HJ_uAsM12] = new HingeJoint(object[upperArm], object[smallMuscle12],
                                      // above HJ between base and upperArm
                                      osg::Vec3(pos[0]-upperArm_length/4-0.01, pos[1], pos[2]),
                                      ((OneAxisJoint*)joint[HJ_BuA])->getAxis1());
    joint[HJ_uAsM12]->init(odeHandle, osgHandle, true);
    // slider joint between smallMuscle11 and smallMuscle12
    joint[SJ_sM1] = new SliderJoint(object[smallMuscle11], object[smallMuscle12],
                                    /*anchor (not used)*/osg::Vec3(0, 0, 0),
                                    osg::Vec3(1,-1, 0));
    joint[SJ_sM1] -> init(odeHandle, osgHandle, /*withVisual*/ false, 0.05);



    /* joints for lower right small Muscle */
    /**************************************/
    // hinge joint between base and smallMuscle21 */
    joint[HJ_BsM21] = new HingeJoint(object[base], object[smallMuscle21],
                                     // same as HJ between base and mainMuscle2l
                                     joint[HJ_BmM21]->getAnchor(),
                                     ((OneAxisJoint*)joint[HJ_BmM21])->getAxis1());
    joint[HJ_BsM21]->init(odeHandle, osgHandle, true);

    // hinge joint between upperArm and smallMuscle22 */
    joint[HJ_uAsM22] = new HingeJoint(object[upperArm], object[smallMuscle22],
                                      // same as HJ between upperArm and smallMuscle12
                                      joint[HJ_uAsM12]->getAnchor(),
                                      ((OneAxisJoint*)joint[HJ_uAsM12])->getAxis1());
    joint[HJ_uAsM22]->init(odeHandle, osgHandle, true);
    // slider joint between smallMuscle21 and smallMuscle22
    joint[SJ_sM2] = new SliderJoint(object[smallMuscle21], object[smallMuscle22],
                                    /*anchor (not used)*/osg::Vec3(0, 0, 0),
                                    osg::Vec3(1,1, 0));
    joint[SJ_sM2] -> init(odeHandle, osgHandle, /*withVisual*/ false, 0.05);





    /* joints for upper left small Muscle */
    /**************************************/
    // hinge joint between lowerArm and smallMuscle31 */
    joint[HJ_lAsM31] = new HingeJoint(object[lowerArm], object[smallMuscle31],
                                      // same as HJ between lowerArm and mainMusclel2
                                      joint[HJ_lAmM12]->getAnchor(),
                                      ((OneAxisJoint*)joint[HJ_lAmM12])->getAxis1());
    joint[HJ_lAsM31]->init(odeHandle, osgHandle, true);
    // hinge joint between upperArm and smallMuscle32 */
    pos=joint[HJ_uAlA]->getAnchor();
    joint[HJ_uAsM32] = new HingeJoint(object[upperArm], object[smallMuscle32],
                                      // below HJ between upperArm and lowerArm
                                      osg::Vec3(pos[0]+upperArm_length/4+0.01, pos[1], pos[2]),
                                      ((OneAxisJoint*)joint[HJ_uAlA])->getAxis1());
    joint[HJ_uAsM32]->init(odeHandle, osgHandle, true);
    // slider joint between smallMuscle31 and smallMuscle32
    joint[SJ_sM3] = new SliderJoint(object[smallMuscle31], object[smallMuscle32],
                                    /*anchor (not used)*/osg::Vec3(0, 0, 0),
                                    osg::Vec3(-1,-1, 0));
    joint[SJ_sM3] -> init(odeHandle, osgHandle, /*withVisual*/ false, 0.05);


    /* joints for upper right small Muscle */
    /***************************************/
    // hinge joint between base and smallMuscle21 */
    joint[HJ_lAsM41] = new HingeJoint(object[lowerArm], object[smallMuscle41],
                                      // same as HJ between lowerArm and mainMuscle22
                                      joint[HJ_lAmM22]->getAnchor(),
                                      ((OneAxisJoint*)joint[HJ_lAmM22])->getAxis1());
    joint[HJ_lAsM41]->init(odeHandle, osgHandle, true);

    // hinge joint between upperArm and smallMuscle42 */
    joint[HJ_uAsM42] = new HingeJoint(object[upperArm], object[smallMuscle42],
                                      //same as HJ between upperArm and smallMuscle32
                                      joint[HJ_uAsM32]->getAnchor(),
                                      ((OneAxisJoint*)joint[HJ_BuA])->getAxis1());
    joint[HJ_uAsM42]->init(odeHandle, osgHandle, true);
    // slider joint between smallMuscle41 and smallMuscle42
    joint[SJ_sM4] = new SliderJoint(object[smallMuscle41], object[smallMuscle42],
                                    /*anchor (not used)*/osg::Vec3(0, 0, 0),
                                    osg::Vec3(-1,1, 0));
    joint[SJ_sM4] -> init(odeHandle, osgHandle, /*withVisual*/ false, 0.05);


    /************************************************************************************/
    created=true;
  };





  /** destroys vehicle and space
   */
  void MuscledArm::destroy(){
    if (created){
      for (int i=0; i<NUMJoints; i++){
        if(joint[i]) delete joint[i];
      }
      for (int i=0; i<NUMParts; i++){
        if(object[i]) delete object[i];
      }
      odeHandle.deleteSpace();
    }
    created=false;
  };



  Primitive* MuscledArm::getMainObject() const {
    return object[hand];
  };


}
