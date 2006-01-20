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
 *   Revision 1.1.4.8  2006-01-20 12:58:14  fhesse
 *   bodies placed correctly
 *
 *   Revision 1.1.4.7  2006/01/13 12:22:07  fhesse
 *   partially working
 *
 *   Revision 1.1.4.6  2006/01/10 16:45:53  fhesse
 *   not working osg version
 *
 *   Revision 1.1.4.5  2006/01/10 09:37:36  fhesse
 *   partially moved to osg
 *
 *   Revision 1.1.4.4  2005/12/16 16:36:04  fhesse
 *   manual control via keyboard
 *   setMotors via dJointAddSliderForce
 *
 *   Revision 1.1.4.3  2005/11/24 16:15:57  fhesse
 *   moved from main branch, sensors improved
 *
 *   Revision 1.3  2005/11/17 16:29:24  fhesse
 *   initial version
 *
 *   Revision 1.2  2005/11/15 12:35:19  fhesse
 *   muscles drawn as muscles, sphere drawn on tip of lower arm
 *
 *   Revision 1.1  2005/11/11 15:37:06  fhesse
 *   preinitial version
 *                                                                 *
 *                                                                         *
 ***************************************************************************/

#include <iostream>
#include <assert.h>

#include "mathutils.h"
#include "muscledarm.h"

namespace lpzrobots{

  MuscledArm::MuscledArm(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const MuscledArmConf& conf):
    OdeRobot(odeHandle, osgHandle), conf(conf){ 

    // prepare name;
    Configurable::insertCVSInfo(name, "$RCSfile$",
				"$Revision$");
 
 
    parentspace=odeHandle.space;
    factorMotors=0.1;
    factorSensors=4.0;//20.0;
    damping=1;

    /*
      initial_pos.x=0.0;
      initial_pos.y=0.0;
      initial_pos.z=0.0;
    */

    sensorno=0;
    if (conf.jointAngleSensors)
      sensorno+=2;
    if (conf.jointAngleRateSensors)
      sensorno+=2;
    if (conf.MuscleLengthSensors)
      sensorno+=6;

    motorno=6;  
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
  void MuscledArm::setMotors(const motor* motors, int motornumber){
  

    /*
    // old execution of motorvalues
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
    //calculating motor force
    force=dist*factorMotors*motors[(int)(i*0.5)-1];
    
    dBodyAddForce (object[i]->getBody, force.x, force.y, force.z);
    dBodyAddForce (object[i+1]->getBody, -force.x, -force.y, -force.z);
    */

  
    //   if (!conf.manualMode) { 
    //     // mode where attached controller is used

    //     /* 
    //     // execution of motor values without slider sensors factor
    //     for (int i=sliderJointM1; i<=sliderJointS4; i++){
    //     // starting with motor[0]
    //     dJointAddSliderForce(joint[i], dJointGetSliderPosition (joint[i]) * factorMotors * motors[i-sliderJointM1]);
    //     }
    //     */
    
    //     // execution of motor values with slider sensors factor
    //     double force;
    //     for(int i=sliderJointM1; i<= sliderJointM2; i++){
    //       force=factorMotors * motors[i-sliderJointM1] * dJointGetSliderPosition (joint[i]) * 5 / (3*SIDE);
    //       dJointAddSliderForce(joint[i], force);
    //     }
    //     for(int i=sliderJointS1; i<= sliderJointS2; i++){
    //       force=factorMotors * motors[i-sliderJointM1] * dJointGetSliderPosition (joint[i]) * 4 / (SIDE);
    //       dJointAddSliderForce(joint[i], force);
    //     }
    //     for(int i=sliderJointS3; i<= sliderJointS4; i++){
    //       force=factorMotors * motors[i-sliderJointM1] * dJointGetSliderPosition (joint[i]) / (0.5*SIDE);
    //       dJointAddSliderForce(joint[i], force);
    //     }
    //   }else{
    //     // manual control (see command in main.cpp)
    //     for(int i=sliderJointM1; i<= sliderJointS4; i++){
    //       dJointAddSliderForce(joint[i], force_[i-sliderJointM1]);
    //     }
    //   }
  };


  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int MuscledArm::getSensors(sensor* sensors, int sensornumber){
    //    int sens_values = (sensornumber < sensorno)? sensornumber : sensorno;
    //
    //   int written=0;
  
    //   if ((conf.jointAngleSensors) && ((sens_values-2)>-1)){
    //     /* hinge joint between upper arm and fixed body: hingeJointFUA (=1) */
    //     /* hinge joint between 2 main arms: hingeJointUALA (=2) */
    //     for (int i=hingeJointFUA; i<hingeJointUALA+1; i++){
    //       sensors[written]=factorSensors*dJointGetHingeAngle(joint[i]);
    //       written++;
    //     }
    //   }
    //   if ((conf.jointAngleRateSensors) && ((sens_values -written -2)>-1)){
    //     for (int i=hingeJointFUA; i<hingeJointUALA+1; i++){
    //       sensors[written]=factorSensors*dJointGetHingeAngleRate(joint[i]);
    //       written++;
    //     }
    //   }
    //   if ((conf.MuscleLengthSensors) && ((sens_values -written -6)>-1)){
    //     for(int i=sliderJointM1; i<= sliderJointM2; i++){
    //       sensors[written] = dJointGetSliderPosition (joint[i]) * 5 / (3*SIDE);
    //       written++;
    //     }
    //     for(int i=sliderJointS1; i<= sliderJointS2; i++){
    //       sensors[written] = dJointGetSliderPosition (joint[i]) * 4 / (SIDE);
    //       written++;
    //     }
    //     for(int i=sliderJointS3; i<= sliderJointS4; i++){
    //       sensors[written] = dJointGetSliderPosition (joint[i]) / (0.5*SIDE);
    //       written++;
    //     }

    // //     double mlen;
    // //     const dReal *p1;
    // //     const dReal *p2;
    // //     for (int i=mainMuscle11; i<smallMuscle42; i+=2){

    // //       p1 = dBodyGetPosition (object[i].body);
    // //       p2 = dBodyGetPosition (object[i+1].body);
    // //       mlen=0;
    // //       //distance between slider joint elements
    // //       mlen+=(p2[0]-p1[0])*(p2[0]-p1[0]);
    // //       mlen+=(p2[1]-p1[1])*(p2[1]-p1[1]);
    // //       mlen+=(p2[2]-p1[2])*(p2[2]-p1[2]);
    // //       mlen=pow(mlen, 1.0/3.0);
    // //       sensors[written]=mlen;
    // //       written++;
    //   }
    //   return written;
    //    return sensornumber;

    sensors[0]= ((HingeJoint*)joint[HJ_BuA])->getPosition1Rate();
    sensors[1]= ((HingeJoint*)joint[HJ_uAlA])->getPosition1Rate();
    return 2;

  };

  /** sets the pose of the vehicle
      @params pose desired 4x4 pose matrix
  */
  void MuscledArm::place(const osg::Matrix& pose){
    // the position of the robot is the center of the base
    // to set the arm on the ground when the z component of the position is 0
    // base_width/2 is added 
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, base_width/2)); 
    create(p2);
  };

  // /** returns position of robot 
  //     @return position robot position in struct Position  
  // */
  // Position MuscledArm::getPosition(){
  //   Position pos;
  
  //   //difference between center of arm and tip of arm 
  //   dReal s[3];
  //   s[0]=0;
  //   s[1]=0;
  //   s[2]=-SIDE*2;
  //   const double* R=dBodyGetRotation(object[lowerArm].body);

  //   //rotation of difference vector
  //   pos.x=s[0]*R[0]+s[1]*R[1]+s[2]*R[2];
  //   pos.y=s[0]*R[4]+s[1]*R[5]+s[2]*R[6];
  //   pos.z=s[0]*R[8]+s[1]*R[9]+s[2]*R[10];
  //   // adding (rotated) difference vector to actual position of arm center
  //   // -> leading to actual position of the tip of the arm
  //   pos = pos + Position(dBodyGetPosition(object[lowerArm].body));

  //   return pos;
  // };

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  int MuscledArm::getSegmentsPosition(vector<Position> &poslist){
    //   assert(created);
    //   for (int i=0; i<NUMParts; i++){
    //     Position pos (dBodyGetPosition(object[i].body));
    //     poslist.push_back(pos);
    //   }   
    return NUMParts;
  };  


  /**
   * draws the arm
   */
  void MuscledArm::update(){
    assert(created); // robot must exist


    object[base]->update();
    object[upperArm]->update();
    object[lowerArm]->update();
    object[hand]->update();

    object[smallMuscle11]->update();
    object[smallMuscle12]->update();
    object[smallMuscle21]->update();
    object[smallMuscle22]->update();
    object[smallMuscle31]->update();
    object[smallMuscle32]->update();
    object[smallMuscle41]->update();
    object[smallMuscle42]->update();

//     joint[fixedJoint]->update();
     joint[HJ_BuA]->update();
     joint[HJ_uAlA]->update();

     joint[HJ_BmM11]->update();
     joint[HJ_lAmM12]->update();
     joint[HJ_BmM21]->update();
     joint[HJ_lAmM22]->update();
     joint[SJ_mM1]->update();
     joint[SJ_mM2]->update();


     joint[HJ_BsM11]->update();
     joint[HJ_uAsM12]->update();

     joint[HJ_BsM21]->update();
     joint[HJ_uAsM22]->update();

     joint[HJ_lAsM31]->update();
     joint[HJ_uAsM32]->update();

     joint[HJ_lAsM41]->update();
     joint[HJ_uAsM42]->update();
           
     joint[SJ_sM1]->update();
     joint[SJ_sM2]->update();
     joint[SJ_sM3]->update();
     joint[SJ_sM4]->update();
    
     for (int i= mainMuscle11; i<smallMuscle11; i++){
       object[i]->update();
     }
//     for (int i=0; i<segmentsno; i++) { 
//       object[i]->update();
//     }
//     for (int i=0; i < 4; i++) { 
//       joint[i]->update();
//     }
  };



  void MuscledArm::doInternalStuff(const GlobalData& globalData){
//     //   if (conf.includeMuscles){
//     //     const dReal ksM1 = 0.5;	// spring constant between mainMuscle11 and mainMuscle12
//     //     const dReal ksM2 = 0.2;	// spring constant between mainMuscle21 and mainMuscle22
//     //     const dReal ksS1 = 0.5;	// spring constant between smallMuscle11 and smallMuscle12
//     //     const dReal ksS2 = 0.2;	// spring constant between smallMuscle21 and smallMuscle22
//     //     const dReal ksS3 = 0.5;	// spring constant between smallMuscle31 and smallMuscle32
//     //     const dReal ksS4 = 0.2;	// spring constant between smallMuscle41 and smallMuscle42

         double k[6];
         k[0] = 0.5;	// spring constant between mainMuscle11 and mainMuscle12
         k[1] = 0.2;	// spring constant between mainMuscle21 and mainMuscle22
         k[2] = 0.5;	// spring constant between smallMuscle11 and smallMuscle12
         k[3] = 0.2;	// spring constant between smallMuscle21 and smallMuscle22
         k[4] = 0.5;	// spring constant between smallMuscle31 and smallMuscle32
         k[5] = 0.2;	// spring constant between smallMuscle41 and smallMuscle42

// 	 damping=0.1;

// 	 double force;
// 	 for(int i=SJ_mM1; i<(SJ_sM4+1); i++){
// 	   //calculating force
// 	   force=((SliderJoint*)joint[i])->getPosition1()  * k[i];
// 	   //damping
// 	   force=force + damping * ((SliderJoint*)joint[i])->getPosition1Rate() ;
// 	   ((SliderJoint*)joint[i])->addForce(0.1*force);
// 	 }
    

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
//     //   }


    //   if(print==1){
    //     dVector3 res;  
    //      for (int i=hingeJointFUA; i<sliderJointM1; i++){
    //        dJointGetHingeAnchor (joint[i], res);
    //        //dJointSetHingeAnchor (joint[hingeJointFM2], 4.7*SIDE, 1.0*SIDE, 1.25*SIDE);
    //        std::cout<<"dJointSetHingeAnchor(joint["<<getJointName(i)<<"], "<<res[0]<<", "<<res[1]<<", "<<res[2]<<"); \n";

    //        dJointGetHingeAxis (joint[i], res);
    //        //dJointSetHingeAxis (joint[hingeJointFM2], 0, 0, 1);
    //        std::cout<<"dJointSetHingeAxis(joint["<<getJointName(i)<<"], "<<res[0]<<", "<<res[1]<<", "<<res[2]<<"); \n";
    //        std::cout<<" \n";
    //      }
    //      for (int i=sliderJointM1; i<sliderJointS4+1; i++){
    //        dJointGetSliderAxis (joint[i], res);
    //        //dJointSetSliderAxis (joint[sliderJointM2],1,0,0);
    //        std::cout<<"dJointSetSliderAxis(joint["<<getJointName(i)<<"], "<<res[0]<<", "<<res[1]<<", "<<res[2]<<"); \n";
    //        std::cout<<" \n";
    //      }

    //      std::cout<<"dReal q[4]; \n";
    //      std::cout<<"Position pos; \n";
    //      for (int i=0; i<NUMParts; i++){
    //        std::cout<<"// "<<getPartName(i)<<"\n";
    //        const dReal* R=dBodyGetQuaternion(object[i].body);
    //        std::cout<<"q[0]="<<R[0]<<";  q[1]="<<R[1]<<";  q[2]="<<R[2]<<";  q[3]="<<R[3]<<";\n";
    //        const dReal* P=dBodyGetPosition(object[i].body);
    //        std::cout<<"pos.x="<<P[0]<<";  pos.y="<<P[1]<<";  pos.z="<<P[2]<<"; \n";
    //      }
    //      print=0;
    //    }
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


  bool MuscledArm::collisionCallback(void *data, dGeomID o1, dGeomID o2){

//     //checks if both of the collision objects are part of the robot
//     if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space) {
    
//       // treat inner collisions in mycallback  => now down with joint stops
//       //dSpaceCollide(arm_space, this, mycallback);

//       int i,n;  
//       const int N = 10;
//       dContact contact[N];
    
//       n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
//       for (i=0; i<n; i++) {
// 	if( contact[i].geom.g1 == object[base]->getGeom() || 
// 	    contact[i].geom.g2 == object[base]->getGeom() ||
// 	    contact[i].geom.g1 == object[upperArm]->getGeom()  || 
// 	    contact[i].geom.g2 == object[upperArm]->getGeom()  || 
// 	    contact[i].geom.g1 == object[lowerArm]->getGeom()  || 
// 	    contact[i].geom.g2 == object[lowerArm]->getGeom() ){ 
// 	  // only treat collisions with fixed body, upper arm or lower arm
// 	  contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
// 	  contact[i].surface.mu = 0.01;
// 	  contact[i].surface.soft_erp = 1;
// 	  contact[i].surface.soft_cfm = 0.00001;

// 	  dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
// 	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
// 	} 
//       }
//       return true;
//     } else {
//       return false;
//     }
 
    return true;
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void MuscledArm::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }
    // create vehicle space and add it to parentspace
    odeHandle.space = dSimpleSpaceCreate(parentspace);

    // create base
    object[base] = new Box(base_width, base_width, base_length);
    object[base] -> init(odeHandle, MASS, osgHandle,Primitive::Geom |Primitive::Body | Primitive::Draw); 

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
      osg::Vec3 pos;      

      // create and place sphere at tip of lower arm
      pos=object[lowerArm]->getPosition();
      object[hand] = new Sphere(lowerArm_width*0.5);
      this->osgHandle.color = Color(0,0,1);
      object[hand] -> init(odeHandle, MASS/20, osgHandle);    
      object[hand] -> setPose(osg::Matrix::translate(pos[0], pos[1]+ lowerArm_length/2, 0)
			      * pose);
      // --------------
      // TODO: change tip to transform object
      //       temporarily positioning of transform object does not work
      //osg::Matrix ps;
      //ps.makeIdentity();
      //Primitive* o1 = new Sphere(lowerArm_width);
      //Primitive* o2 = new Transform(object[lowerArm], o1, 
      //			    osg::Matrix::translate(0, lowerArm_length*0.5, 0) * ps);
      //o2->init(odeHandle, /*mass*/0, osgHandle, /*withBody*/ false);
      // --------------    


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
	for (int i=0; i<NUMParts; i++){
	  if(object[i]) delete object[i];
	}
	for (int i=0; i<NUMJoints; i++){
	  if(joint[i]) delete joint[i];
	}
	dSpaceDestroy(odeHandle.space);
      }
      created=false;
    };



    Primitive* MuscledArm::getMainObject() const {
      return object[hand];  
    };

    Configurable::paramlist MuscledArm::getParamList() const{
      paramlist list;
      list.push_back(pair<paramkey, paramval> (string("factorMotors"), factorMotors));
      list.push_back(pair<paramkey, paramval> (string("factorSensors"), factorSensors));
      list.push_back(pair<paramkey, paramval> (string("damping"), damping));
      list.push_back(pair<paramkey, paramval> (string("print"), print));
      return list;
    };

    Configurable::paramval MuscledArm::getParam(const paramkey& key) const{
      if(key == "factorMotors") return factorMotors; 
      else if(key == "factorSensors") return factorSensors; 
      else if(key == "damping") return damping; 
      else if(key == "print") return print; 
      else  return Configurable::getParam(key) ;
    }

    bool MuscledArm::setParam(const paramkey& key, paramval val){
      if(key == "factorMotors") factorMotors=val;
      else if(key == "factorSensors") factorSensors = val; 
      else if(key == "damping") damping = val; 
      else if(key == "print") print = val; 
      else return Configurable::setParam(key, val);
      return true;
    }

    std::string MuscledArm::getJointName(int j) const{
      assert( (j>-1) && (j<=NUMJoints) );
      std::string name;
      switch (j)
	{
	case  0: return "fixedJoint"; break;
	case  1: return "hingeJointFUA"; break; 
	case  2: return "hingeJointUALA"; break; 
	case  3: return "hingeJointFM1"; break;
	case  4: return "hingeJointFM2"; break; 
	case  5: return "hingeJointFS1"; break;
	case  6: return "hingeJointFS2"; break;
	case  7: return "hingeJointUAS1"; break;
	case  8: return "hingeJointUAS2"; break;
	case  9: return "hingeJointUAS3"; break;
	case 10: return "hingeJointUAS4"; break;
	case 11: return "hingeJointLAM1"; break;
	case 12: return "hingeJointLAM2"; break;
	case 13: return "hingeJointLAS3"; break;
	case 14: return "hingeJointLAS4"; break;
	case 15: return "sliderJointM1"; break;
	case 16: return "sliderJointM2"; break;
	case 17: return "sliderJointS1"; break; 
	case 18: return "sliderJointS2"; break;
	case 19: return "sliderJointS3"; break;
	case 20: return "sliderJointS4"; break;
	case 21: return "fixedJointHand"; break;
	case 22: return "NUMJoints"; break;
	}  
    }

    std::string MuscledArm::getPartName(int j) const{
      assert( (j>-1) && (j<=NUMParts) );
      std::string name;
      switch (j)
	{
	case  0: return "base"; break;
	case  1: return "upperArm"; break; 
	case  2: return "lowerArm"; break; 
	case  3: return "mainMuscle11"; break;
	case  4: return "mainMuscle12"; break; 
	case  5: return "mainMuscle21"; break;
	case  6: return "mainMuscle22"; break;
	case  7: return "smallMuscle11"; break;
	case  8: return "smallMuscle12"; break;
	case  9: return "smallMuscle21"; break;
	case 10: return "smallMuscle22"; break;
	case 11: return "smallMuscle31"; break;
	case 12: return "smallMuscle32"; break;
	case 13: return "smallMuscle41"; break;
	case 14: return "smallMuscle42"; break;
	case 15: return "hand"; break;
	case 16: return "NUMParts"; break;
	}  
    }
  }
