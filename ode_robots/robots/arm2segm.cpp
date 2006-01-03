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
 *   Revision 1.6.4.3  2006-01-03 10:01:46  fhesse
 *   moved to osg
 *
 *   Revision 1.6.4.2  2005/11/15 12:29:25  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.6.4.1  2005/11/14 17:37:16  martius
 *   moved to selforg
 *
 *   Revision 1.6  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/

#include "arm2segm.h"

namespace lpzrobots{

  Arm2Segm::Arm2Segm(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const Arm2SegmConf armConf):
    OdeRobot(odeHandle, osgHandle), conf(armConf){ 

    // prepare name;
    Configurable::insertCVSInfo(name, "$RCSfile$",
				"$Revision$");

    parentspace=odeHandle.space;

    created=false;


    speed=1.0;
    factorSensors=2.0;

    sensorno=conf.segmentsno; 
    motorno=conf.segmentsno;

//     for (int i=0; i<segmentsno; i++){
//       old_angle[i]=0.0;
//     }
 
    this->osgHandle.color = Color(1, 0, 0, 1.0f);

  };

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void Arm2Segm::setMotors(const motor* motors, int motornumber){
    //   int len = (motornumber < motorno)? motornumber : motorno;
    //   double old_vel, new_vel;
    //   for (int i=0; i<len; i++){ 
    //     old_vel = dJointGetAMotorParam( jm[i] , dParamVel);
    //     new_vel = old_vel + avgMotor*(motors[i]-old_vel);
    //     dJointSetAMotorParam ( jm[i] , dParamVel , new_vel * speed );
    //     //if (i==2)
    //     //dJointSetAMotorParam ( jm[i] , dParamVel , winkelgeschwindigkeit*speed*0.75 );
    //   }

    assert(created); // robot must exist
    // the number of controlled motors is minimum of
    // "number of motorcommands" (motornumber) and 
    // "number of motors inside the robot" (motorno)
    int len = (motornumber < motorno)? motornumber : motorno;

    // for each motor the motorcommand (between -1 and 1) multiplied with speed
    // is set and the maximal force to realize this command are set
    for (int i=0; i<len; i++){ 
      //      joints[i]->setParam(dParamVel, motors[i]*speed);       
      //      joints[i]->setParam(dParamFMax2, max_force);
      amotors[i]->set(1, motors[i]*speed);
    }


    // another possibility is to set half of the difference between last set speed
    // and the actual desired speed as new speed; max_force is also set
    /*
      double tmp;
      int len = (motornumber < motorno)? motornumber : motorno;
      for (int i=0; i<len; i++){ 
      tmp=dJointGetHinge2Param(joints[i],dParamVel2);
      dJointSetHinge2Param(joints[i],dParamVel2,tmp + 0.5*(motors[i]*speed-tmp) );       
      dJointSetHinge2Param (joints[i],dParamFMax2,max_force);
      }
    */

  };


  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Arm2Segm::getSensors(sensor* sensors, int sensornumber){
//     int len = (sensornumber < sensorno)? sensornumber : sensorno;
//     double w;
//     for ( int n = 0; n < len; n++ ){
//       //     w=dJointGetAMotorAngle (jm[n],0)-old_angle[n];
//       //     old_angle[n] = dJointGetAMotorAngle ( jm[n] , 0 );
//       //     sensors[n]=w*factorSensors;      
//       //     if ( ( w < -M_PI ) || ( w >  M_PI ) ){
//       //       if ( w > M_PI ){
//       // 	sensors[n] = -(2*M_PI - w);
//       //       }
//       //       if ( w < -M_PI ){
//       // 	sensors[n] = (2*M_PI + w);
//       //       }
//       //     }
//     }
//     return len;

    assert(created); // robot must exist

    // the number of sensors to read is the minimum of
    // "number of sensors requested" (sensornumber) and 
    // "number of sensors inside the robot" (sensorno)
    int len = (sensornumber < sensorno)? sensornumber : sensorno;

    // for each sensor the anglerate of the joint is red and scaled with 1/speed 
    for (int i=0; i<len; i++){
      //      sensors[i]=((HingeJoint*)joints[i])->getPosition1Rate();
      sensors[i]=amotors[i]->get(1);
      sensors[i]/=speed;  //scaling
    }
    // the number of red sensors is returned 
    return len;

  };

  /** sets the vehicle to position pos, sets color to c, and creates robot if necessary
      @params pos desired position of the robot in struct Position
      @param c desired color for the robot in struct Color
  */
  void Arm2Segm::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // SOCKEL_HOEHE* 0.5 is added (without this the wheels and half of the robot will be in the ground)    
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.SOCKEL_HOEHE* 0.5)); 
    create(p2);
  };


  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  int Arm2Segm::getSegmentsPosition(vector<Position> &poslist){
    for (int i=0; i<conf.segmentsno; i++){
      Pos p = objects[i]->getPosition();
      poslist.push_back(p.toPosition());
    } 
    return conf.segmentsno;
  };  


  void Arm2Segm::update(){
    assert(created); // robot must exist
    for (vector<Primitive*>::iterator i = objects.begin(); i!=objects.end(); i++) { 
      if (*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!=joints.end(); i++) { 
      if (*i) (*i)->update();
    }
  };

  void Arm2Segm::mycallback(void *data, dGeomID o1, dGeomID o2){
    //Arm2Segm* me = (Arm2Segm*)data;  
    // if(isGeomInObjectList(me->object, me->segmentsno, o1) && isGeomInObjectList(me->object, me->segmentsno, o2)){
    //   return;
    // }
  };

  void Arm2Segm::doInternalStuff(const GlobalData& globalData){};
  bool Arm2Segm::collisionCallback(void *data, dGeomID o1, dGeomID o2){
    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space){
      // TODO: internal space collisions should go to doInternalStuff() (see hurlingsnake.cpp)
      // brauchmer doch gar nicht (gibt doch keine internen Kollisionen)
      //dSpaceCollide(odeHandle.space, this, mycallback);
      bool colwithme;  
      int i,n;  
      const int N = 10;
      dContact contact[N];
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++){

	colwithme = false;  
	for (int j=0; j< conf.segmentsno; j++){
	  if( contact[i].geom.g1 == objects[j]->getGeom() || contact[i].geom.g2 == objects[j]->getGeom()){
	    colwithme = true;
	  }
	}
	//       if( contact[i].geom.g1 == objects[0].geom || contact[i].geom.g2 == objects[0].geom ||
	// 	  contact[i].geom.g1 == objects[1].geom || contact[i].geom.g2 == objects[1].geom || 
	// 	  contact[i].geom.g1 == objects[2].geom || contact[i].geom.g2 == objects[2].geom ){
	// 	colwithme = true;
	//       }
	if( colwithme){
	  contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	    dContactSoftERP | dContactSoftCFM | dContactApprox1;
	  contact[i].surface.slip1 = 0.005;
	  contact[i].surface.slip2 = 0.005;
	  contact[i].surface.mu = 0.5;
	  contact[i].surface.soft_erp = 0.9;
	  contact[i].surface.soft_cfm = 0.001;
	  dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	
	}
      }
      return true;
    }
    return false;
  };


  Primitive* Arm2Segm::getMainPrimitive() const{ 
    return objects[conf.segmentsno-1]; 
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void Arm2Segm::create(const osg::Matrix& pose){

    if (created) {
      destroy();
    }
  
    // create vehicle space and add it to parentspace
    odeHandle.space = dSimpleSpaceCreate (parentspace);

    Primitive* o = new Box(conf.SOCKEL_LAENGE, conf.SOCKEL_BREITE, conf.SOCKEL_HOEHE);
    o -> init(odeHandle, conf.base_mass, osgHandle);
    // rotate and place body (here by 90° around the y-axis)
    o->setPose( pose);
    //o->getOSGPrimitive()->setTexture("Images/wood.rgb");
    objects.push_back(o);

     for (int i=0; i<(conf.segmentsno-1); i++){
       o = new Box(conf.ARMLAENGE, conf.ARMDICKE, conf.ARMDICKE);
       o -> init(odeHandle, conf.arm_mass, osgHandle);
       o -> 
	 setPose(osg::Matrix::translate(osg::Vec3(
						  (i+0.5)*conf.ARMLAENGE - (i+1)*conf.gelenkabstand,
						  (i+1)*conf.ARMABSTAND + (i+0.5)*conf.ARMDICKE + 0.5*conf.SOCKEL_BREITE,
						  0)
					) * pose);
       objects.push_back(o);
     }

     Pos p1(objects[0]->getPosition());
     HingeJoint* j = new HingeJoint(0, objects[0], p1, osg::Vec3(0,0,1));
     j -> init(odeHandle, osgHandle,/*withVisual*/true);
     joints.push_back(j);
     AngularMotor1Axis* a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[0], conf.max_force);
     amotors.push_back(a);

     Pos p2(objects[1]->getPosition());
     p1[1]=(p1[1]+p2[1])/2;
     j = new HingeJoint(objects[0], objects[1], p1, osg::Vec3(0,1,0));
     j -> init(odeHandle, osgHandle,/*withVisual*/true);
     joints.push_back(j);
     a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[1], conf.max_force);
     amotors.push_back(a);

     for (int i=2; i<conf.segmentsno; i++) {
       Pos po1(objects[i-1]->getPosition());
       Pos po2(objects[i]->getPosition());
       Pos po3( (po1+po2)/2);  
       j = new HingeJoint(objects[i-1], objects[i], po3, osg::Vec3(0,1,0));
       j -> init(odeHandle, osgHandle,/*withVisual*/true);
       joints.push_back(j);
       a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[i], conf.max_force);
       amotors.push_back(a);
     }


    //TODO:
    // - joint between base and world is missing
    // - motors missing;
   //  for (int i=0; i<segmentsno; i++) {
//       Pos p1(object[i]->getPosition());
//       Pos p2(object[i+1]->getPosition());
//       joint[i] = new HingeJoint(object[i],object[i+1], (p1+p2)/2 );
//       joint[i]->init(odeHandle, osgHandle, true, ARMDICKE*1.5);
//     }

    //   //*************Koerperdefinitionsabschnitt**************
    //   //Sockel fuer die ROtation
    //   object[0].body = dBodyCreate ( world );
    //   dBodySetPosition ( object[0].body , 
    // 		     pos.x + 0.5*SOCKEL_LAENGE , 
    // 		     pos.y + 0.001 ,  
    // 		     pos.z + SOCKEL_HOEHE* 0.5+ 0.01 );
    //   object[1].body = dBodyCreate ( world );
    //   dBodySetPosition ( object[1].body , 
    // 		     pos.x + 0.5*ARMLAENGE + gelenkabstand , 
    // 		     pos.y + ARMABSTAND + ARMDICKE , 
    // 		     pos.z + SOCKEL_HOEHE*0.5 + 0.01 );

    //   //Arme
    //   for ( int n = 2; n < armanzahl+1; n=n+1 )
    //     {
    //       object[n].body = dBodyCreate ( world );
    //       dBodySetPosition ( object[n].body , 
    // 			 dBodyGetPositionAll ( object[n-1].body , 1 ) + ARMLAENGE - gelenkabstand, 
    // 			 dBodyGetPositionAll ( object[n-1].body , 2 ) + ARMDICKE + ARMABSTAND,  
    // 			 dBodyGetPositionAll ( object[n-1].body , 3 ) );
      
    //     }

    //   dMass masse;
    //   //Aufbau der Massenverteilungsmatrix (hier fuer eine Box)
    //   dMassSetBox ( &masse , 1 , SOCKEL_LAENGE , SOCKEL_BREITE , SOCKEL_HOEHE );
    //   dMassAdjust ( &masse , SOCKEL_MASSE );
    //   dBodySetMass ( object[0].body , &masse );

    //   dMassSetBox ( &masse , 1 , ARMLAENGE , ARMDICKE , ARMDICKE );
    //   dMassAdjust ( &masse , ARMMASSE );
    //   dBodySetMass ( object[1].body , &masse );
  
    //   //Aenderung der Masse des Koerpers, aber beibehaltung der Massenverteilungsmatrix
    //   for ( int n = 2; n < armanzahl+1; n=n+1 ){
    //     dMassSetBox ( &masse , 1 , ARMLAENGE , ARMDICKE , ARMDICKE );
    //     dMassAdjust ( &masse , ARMMASSE );
    //     dBodySetMass ( object[n].body , &masse );
    //   }

    //   //**************Huellenfestlegungsabschnitt*************
    //   //Rotationssockel
    //   object[0].geom = dCreateBox ( arm_space , SOCKEL_LAENGE , SOCKEL_BREITE , SOCKEL_HOEHE );
    //   dGeomSetBody ( object[0].geom , object[0].body );
    //   //Arme
    //   object[1].geom = dCreateBox ( arm_space , ARMLAENGE , ARMDICKE , ARMDICKE );
    //   dGeomSetBody ( object[1].geom , object[1].body );
  
    //   for ( int n = 2; n < armanzahl+1; n=n+1 ){
    //     object[n].geom = dCreateBox ( arm_space , ARMLAENGE , ARMDICKE , ARMDICKE );
    //     dGeomSetBody ( object[n].geom , object[n].body );
    //   }


    //   //***************Motordefinitionsabschnitt**************
  
    //   jm[0] = dJointCreateAMotor ( world , 0 );
    //   dJointAttach ( jm[0] , object[0].body , 0 );
    //   dJointSetAMotorMode ( jm[0] , dAMotorEuler );
    //   //Dies sind die beiden festen Axen
    //   dJointSetAMotorAxis ( jm[0] , 0 , 1 , 0 , 0 , 1 );
    //   dJointSetAMotorAxis ( jm[0] , 2 , 2 , 1 , 0 , 0 );
    //   dJointSetAMotorParam ( jm[0] , dParamFMax , max_force*7/*20*/ );

    //   jm[1] = dJointCreateAMotor ( world , 0 );
    //   dJointAttach ( jm[1] , object[1].body , object[0].body );
    //   dJointSetAMotorMode ( jm[1] , dAMotorEuler );
    //   //Dies sind die beiden festen Axen
    //   dJointSetAMotorAxis ( jm[1] , 0 , 1 , 0 , 1 , 0 );
    //   dJointSetAMotorAxis ( jm[1] , 2 , 2 , 0 , 0 , 1 );
    //   dJointSetAMotorParam ( jm[1] , dParamFMax , max_force*2 /*10*/ );





    //   for ( int n = 2; n < armanzahl+1; n=n+1 ){
    //     jm[n] = dJointCreateAMotor ( world , 0 );
    //     dJointAttach ( jm[n] , object[n].body , object[n-1].body );
    //     dJointSetAMotorMode ( jm[n] , dAMotorEuler );
    //     //Dies sind die beiden festen Axen
    //     dJointSetAMotorAxis ( jm[n] , 0 , 1 , 0 , 1 , 0 );
    //     dJointSetAMotorAxis ( jm[n] , 2 , 2 , 0 , 0 , 1 );
    //     dJointSetAMotorParam ( jm[n] , dParamFMax , max_force );
    //   }

    //   //*****************Join-Generierungsabschnitt***********
    //     //Sockel-Rotationsgelenk

    //   //  j[0] = dJointCreateHinge(world, 0);
    //   joint[0] = dJointCreateHinge (world,0);
    //   dJointAttach ( joint[0] , object[0].body , 0 );
    //   dJointSetHingeAnchor ( joint[0] , dBodyGetPositionAll ( object[0].body , 1 ) , 
    // 			 dBodyGetPositionAll ( object[0].body , 2 ) , 
    // 			 dBodyGetPositionAll ( object[0].body , 3 ) );
    //   dJointSetHingeAxis ( joint[0] , 0 , 0 , 1 );

    //   //Erstes Armgelenk (am Rotationssockel befestigt)
    //   joint[1] = dJointCreateHinge ( world , 0 );
    //   dJointAttach ( joint[1] , object[1].body , object[0].body );
    //   dJointSetHingeAnchor ( joint[1] , dBodyGetPositionAll ( object[0].body , 1 ) , 
    // 			 dBodyGetPositionAll ( object[0].body , 2 ) , 
    // 			 dBodyGetPositionAll ( object[1].body , 3 ) );
    //   dJointSetHingeAxis ( joint[1] , 0 , 1 , 0 );

    //   for ( int n = 2; n < armanzahl+1; n=n+1 ){
    //     joint[n] = dJointCreateHinge ( world , 0 );
    //     dJointAttach ( joint[n] , object[n].body , object[n-1].body );
    //     dJointSetHingeAnchor ( joint[n] , dBodyGetPositionAll ( object[n-1].body , 1 ) 
    // 			   + (dBodyGetPositionAll ( object[n].body , 1 ) 
    // 			      - dBodyGetPositionAll ( object[n-1].body , 1 ))/2 , 
    // 			   dBodyGetPositionAll ( object[n].body , 2 ) ,  
    // 			   dBodyGetPositionAll ( object[n].body , 3 ) );
    //     dJointSetHingeAxis ( joint[n] , 0 , 1 , 0 );
    //   }



  
    created=true;
  }; 


  /** destroys vehicle and space
   */
  void Arm2Segm::destroy(){
    if (created){
      for (vector<Primitive*>::iterator i=objects.begin(); i!=objects.end(); i++){
	if (*i) delete *i;
      }
      objects.clear();
      for (vector<Joint*>::iterator i=joints.begin(); i!=joints.end(); i++){
	if (*i) delete *i;
      }
      joints.clear();
      for (vector<AngularMotor1Axis*>::iterator i=amotors.begin(); i!=amotors.end(); i++){
	if (*i) delete *i;
      }
      amotors.clear();
      dSpaceDestroy(odeHandle.space);
    }
    created=false;
  };



  Configurable::paramlist Arm2Segm::getParamList() const{
    paramlist list;
    list.push_back(pair<paramkey, paramval> (string("speed"), speed));
    list.push_back(pair<paramkey, paramval> (string("factorSensors"), factorSensors));
    return list;
  }

  Configurable::paramval Arm2Segm::getParam(const paramkey& key) const{
    if(key == "speed") return speed; 
    else if(key == "factorSensors") return factorSensors; 
    else  return Configurable::getParam(key) ;
  };

  bool Arm2Segm::setParam(const paramkey& key, paramval val){
    if(key == "speed") speed=val;
    else if(key == "factorSensors") factorSensors = val; 
    else return Configurable::setParam(key, val);
    return true;
  };

}
