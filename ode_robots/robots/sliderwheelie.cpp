/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.7  2006-09-21 10:21:33  robot8
 *   - parameters of structure changed
 *
 *   Revision 1.6  2006/09/21 09:38:02  robot8
 *   *** empty log message ***
 *
 *   Revision 1.5  2006/09/21 08:14:55  martius
 *   with sliders inside a segment
 *
 *   Revision 1.4  2006/09/20 12:56:17  martius
 *   Snakes have CreateSegment
 *
 *   Revision 1.3  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.4  2006/06/26 20:33:50  robot5
 *   Joint adjustments.
 *
 *   Revision 1.1.2.3  2006/06/25 21:57:20  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.2  2006/06/20 07:18:29  robot3
 *   -added cvs log
 *   -changed some behaviour of wheelie
 *
 *
 ***************************************************************************/

#include "sliderwheelie.h"
using namespace std;

namespace lpzrobots {

  SliderWheelie::SliderWheelie(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			       const SliderWheelieConf& conf, 
			       const std::string& name, const std::string& revision) 
    : OdeRobot(odeHandle, osgHandle, name, (revision.empty()? "$Id$" : revision))
	  , conf(conf)
  {
        created=false;
  }
	
  SliderWheelie::~SliderWheelie() {
    if(created) destroy();
  }
	

  void SliderWheelie::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)    
    create(pose * osg::Matrix::translate(osg::Vec3(0, 0, 0.5*conf.segmLength*conf.segmNumber/M_PI 
						   + conf.segmDia/2))); 
  }

  void SliderWheelie::update(){
    assert(created); // robot must exist
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }
  }

  void SliderWheelie::doInternalStuff(const GlobalData& global){
    if(created){
      // mycallback is called for internal collisions! Only once per step
      //      dSpaceCollide(odeHandle.space, this, mycallback);
    }
  }

  void SliderWheelie::mycallback(void *data, dGeomID o1, dGeomID o2)
  {
    SliderWheelie* me = (SliderWheelie*) data;
    int i=0;
    int o1_index= -1;
    int o2_index= -1;
    for (vector<Primitive*>::iterator n = me->objects.begin(); n!= me->objects.end(); n++, i++) {      
      if( (*n)->getGeom() == o1)
	o1_index=i;
      if( (*n)->getGeom() == o2)
	o2_index=i;
    }

    if(o1_index >= 0 && o2_index >= 0 && abs(o1_index - o2_index) > 1 &&
       !(o1_index == 0 && o2_index == me->conf.segmNumber-1)) {
      // internal collisions
      int i,n;  
      const int N = 10;
      dContact contact[N];  
      n = dCollide (o1, o2, N, &contact[0].geom, sizeof(dContact));	  
      for (i=0; i<n; i++) {
	contact[i].surface.mode = 0;
	contact[i].surface.mu = 0;
	contact[i].surface.mu2 = 0;
	dJointID c = dJointCreateContact( me->odeHandle.world, me->odeHandle.jointGroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;     
      }
    }
  } 


  void SliderWheelie::setMotors(const motor* motors, int motornumber) {
   assert(created);
   unsigned int len = min(motornumber, getMotorNumber())/2;
   // controller output as torques 

   for(unsigned int i=0; (i<len) && (i<hingeServos.size()); i++) {
    hingeServos[i]->set(motors[i]);
   }

   for(unsigned int i=0; (i<len) && (i<sliderServos.size()); i++) {
    sliderServos[i]->set(motors[i]);
   }
  }

  int SliderWheelie::getSensors(sensor* sensors, int sensornumber) {
   assert(created);
   unsigned int len=min(sensornumber, getSensorNumber());
   // get the hingeServos

   for(unsigned int n=0; (n<len) && (n<hingeServos.size()); n++) {
    sensors[n] = hingeServos[n]->get();
   }

   for(unsigned int n=0; (n<len) && (n<sliderServos.size()); n++) {
    sensors[n] = sliderServos[n]->get();
   }
   return len;
  }

  bool SliderWheelie::collisionCallback(void *data, dGeomID o1, dGeomID o2)
  {
    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space ){
      int i,n;  
      const int N = 20;
      dContact contact[N];
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++){
	//      contact[i].surface.mode = dContactMu2 | dContactSlip1 | dContactSlip2 |
	//	dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |	
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.slip1 = 0.001;
	contact[i].surface.slip2 = 0.001;
	contact[i].surface.mu = conf.frictionGround; //*10;
	//      contact[i].surface.mu2 = conf.frictionGround;
	contact[i].surface.soft_erp = 0.9;
	contact[i].surface.soft_cfm = 0.01;
	
	dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
      }
      return true;
    }
    return false;
  }


  void SliderWheelie::create(const osg::Matrix& pose) {
    if (created) {
      destroy();
    }
    
    odeHandle.space = dSimpleSpaceCreate (parentspace);
	
    vector<Pos> ancors;
    // annular positioning
    for(int n = 0; n < conf.segmNumber; n++) {
      osg::Matrix m = osg::Matrix::rotate(M_PI*2*n/conf.segmNumber, 0, -1, 0) * pose;
      if(n%2==0){
	
	Primitive* p1 = new Box(conf.segmDia/2, conf.segmDia*4, conf.segmLength/2);
	p1->init(odeHandle, conf.segmMass/2 , osgHandle);
	p1->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
		    osg::Matrix::translate(-conf.segmLength/4,0,
					   -0.5*conf.segmLength*conf.segmNumber/M_PI) * m );
	p1->setTexture("Images/wood.rgb");
	objects.push_back(p1);

	Primitive* p2 = new Box(conf.segmDia/2, conf.segmDia*4, conf.segmLength/2);
	
	p2->init(odeHandle, conf.segmMass/2 , osgHandle);
	p2->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
		    osg::Matrix::translate(conf.segmLength/4,0,
					   -0.5*conf.segmLength*conf.segmNumber/M_PI) * m );	
	p2->setTexture("Images/dusty.rgb");
	objects.push_back(p2);

	const Pos& pos1(p1->getPosition());
	const Pos& pos2(p2->getPosition());
	
	SliderJoint* j = new SliderJoint(p1, p2,
					 (pos1 + pos2)/2,
					 Axis(1,0,0)*m);
	j->init(odeHandle, osgHandle, true, conf.segmDia);
	
	// setting stops at slider joints
	j->setParam(dParamLoStop, -conf.segmLength*conf.sliderLength);
	j->setParam(dParamHiStop,  conf.segmLength*conf.sliderLength/2);
	joints.push_back(j);
	
	SliderServo* servo = new SliderServo(j, 
					     -conf.segmLength*conf.sliderLength, 
					     conf.segmLength*conf.sliderLength/2, 
					     conf.motorPower);
	sliderServos.push_back(servo);	
      }else{
	Primitive* p1 = new Box(conf.segmDia/2, conf.segmDia*4*( (n+1)%4 ==0 ? 2 : 1), conf.segmLength);
	p1->init(odeHandle, conf.segmMass * ( (n+1)%4 ==0 ? 2 : 1), osgHandle);
	p1->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
		    osg::Matrix::translate(0,0,-0.5*conf.segmLength*conf.segmNumber/M_PI) * m );
	p1->setTexture("Images/whitemetal_farbig_small.rgb");
	objects.push_back(p1);
      }
      ancors.push_back(Pos(conf.segmLength/2,0,-0.5*conf.segmLength*conf.segmNumber/M_PI) * m);
     }


   //***************** hinge joint definition***********
    int i = 0;
    for(int n=0; n < conf.segmNumber-1; n++, i++) {            
      if(n%2==0){
	i++;	
      }
      int o1 = i;
      int o2 = i+1;
//      const Pos& p1(objects[o1]->getPosition());
//      const Pos& p2(objects[o2]->getPosition());
      
      HingeJoint* j = new HingeJoint(objects[o1], objects[o2],
				     ancors[n],
				     Axis(0,1,0)*pose);
      j->init(odeHandle, osgHandle, true, conf.segmDia*2);
      
      // setting stops at hinge joints
            j->setParam(dParamLoStop, -conf.jointLimit);
            j->setParam(dParamHiStop,  conf.jointLimit);
      joints.push_back(j);
      
      HingeServo* servo = new HingeServo(j, -conf.jointLimit, conf.jointLimit, conf.motorPower);
      hingeServos.push_back(servo);

    }

   // connecting first and last segment
   const Pos& p1((*objects.rbegin())->getPosition());
   const Pos& p2((*objects.begin())->getPosition());

   HingeJoint* j = new HingeJoint((*objects.rbegin()), (*objects.begin()),
				  (p1 + p2)/2,
 				  Axis(0,1,0)*pose);
   j->init(odeHandle, osgHandle, true, conf.segmDia/2*1.02);

   // setting stops at hinge joints
   j->setParam(dParamLoStop, -conf.jointLimit);
   j->setParam(dParamHiStop,  conf.jointLimit);
   joints.push_back(j);

   HingeServo* servo = new HingeServo(j, -conf.jointLimit, conf.jointLimit, conf.motorPower);
   hingeServos.push_back(servo);


   created=true;

  }

  /** destroys vehicle and space
   */
  void SliderWheelie::destroy() {
   if(created) {
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++) {
	if(*i) delete *i;
      }
      objects.clear();
      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
	if(*i) delete *i;
      }
      joints.clear();
      for (vector<AngularMotor*>::iterator i = frictionmotors.begin(); i!= frictionmotors.end(); i++){
	if(*i) delete *i;
      }
      frictionmotors.clear();
      dSpaceDestroy(odeHandle.space);

      for (vector<HingeServo*>::iterator i = hingeServos.begin(); i!= hingeServos.end(); i++) {
	if(*i) delete *i;
      }

      for (vector<SliderServo*>::iterator i = sliderServos.begin(); i!= sliderServos.end(); i++) {
	if(*i) delete *i;
      }
      hingeServos.clear();
      sliderServos.clear();
   }
  }

  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  Configurable::paramlist SliderWheelie::getParamList() const{
    paramlist list;
    list += pair<paramkey, paramval> (string("frictionground"), conf.frictionGround);
    list += pair<paramkey, paramval> (string("frictionjoint"), conf.frictionJoint);
    list += pair<paramkey, paramval> (string("motorpower"),   conf.motorPower);
    list += pair<paramkey, paramval> (string("sensorfactor"), conf.sensorFactor);
    return list;
  }
  
  
  Configurable::paramval SliderWheelie::getParam(const paramkey& key) const{    
    if(key == "frictionground") return conf.frictionGround; 
    else if(key == "frictionjoint") return conf.frictionJoint; 
    else if(key == "motorpower") return conf.motorPower; 
    else if(key == "sensorfactor") return conf.sensorFactor; 
    else  return Configurable::getParam(key) ;
  }
  
  bool SliderWheelie::setParam(const paramkey& key, paramval val){    
    if(key == "frictionground") conf.frictionGround = val; 
    else if(key == "motorpower") { 
      conf.motorPower = val; 

      for(vector<HingeServo*>::iterator i=hingeServos.begin(); i!=hingeServos.end(); i++) {
	if(*i) (*i)->setPower(conf.motorPower);
      }

      for(vector<SliderServo*>::iterator i=sliderServos.begin(); i!=sliderServos.end(); i++) {
	if(*i) (*i)->setPower(conf.motorPower);
      }      
    }
    else if(key == "sensorfactor") conf.sensorFactor = val; 
    else if(key == "frictionjoint") { 
      conf.frictionJoint = val; 
      for (vector<AngularMotor*>::iterator i = frictionmotors.begin(); 
	   i!= frictionmotors.end(); i++){
	if (*i) (*i)->setPower(conf.frictionJoint);	
      }         
    } else 
      return Configurable::setParam(key, val);    
    return true;
  }


}
