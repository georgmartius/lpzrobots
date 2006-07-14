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
 *   Revision 1.7  2006-07-14 12:23:38  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.6.4.8  2006/06/25 17:00:31  martius
 *   Id
 *
 *   Revision 1.6.4.7  2006/06/25 16:57:11  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.6.4.6  2006/01/05 13:50:31  fhesse
 *   started to add tip at the end
 *
 *   Revision 1.6.4.5  2006/01/04 14:45:10  fhesse
 *   hingejoint axis multiplied with pose (in hinge joiunt constructor)
 *
 *   Revision 1.6.4.4  2006/01/03 13:18:51  fhesse
 *   cleaned up
 *   TODO: in the long run robot disappears (huge sensorvalues)
 *
 *   Revision 1.6.4.3  2006/01/03 10:01:46  fhesse
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
    OdeRobot(odeHandle, osgHandle,"Arm2Segm", "$Id$"), conf(armConf){ 

    parentspace=odeHandle.space;

    speed=1.0;
    factorSensors=2.0;
    sensorno=conf.segmentsno; 
    motorno=conf.segmentsno;
    this->osgHandle.color = Color(1, 0, 0, 1.0f);

    created=false;
  };

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void Arm2Segm::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist
    // the number of controlled motors is minimum of
    // "number of motorcommands" (motornumber) and 
    // "number of motors inside the robot" (motorno)
    int len = (motornumber < motorno)? motornumber : motorno;

    // for each motor the motorcommand (between -1 and 1) multiplied with speed
    // is set (maximal force defined by amotors, see create() below)
    for (int i=0; i<len; i++){ 
      amotors[i]->set(1, motors[i]*speed);
    }
  
    // another possibility is to set half of the difference between last set speed
    // and the actual desired speed as new speed;
    /*
    double tmp;
    for (int i=0; i<len; i++){ 
      tmp=amotors[i]->get(1);
      amotors[i]->set(1,tmp + 0.5*(motors[i]*speed-tmp) );
    }
    */
  };


  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Arm2Segm::getSensors(sensor* sensors, int sensornumber){
    assert(created); // robot must exist

    // the number of sensors to read is the minimum of
    // "number of sensors requested" (sensornumber) and 
    // "number of sensors inside the robot" (sensorno)
    int len = (sensornumber < sensorno)? sensornumber : sensorno;

    // for each sensor the anglerate of the joint is red and scaled with 1/speed 
    for (int i=0; i<len; i++){
      sensors[i]=amotors[i]->get(1);  // is equal to: ((HingeJoint*)joints[i])->getPosition1Rate()
      // or read angle of each joint:
      // sensors[i]=((HingeJoint*)joints[i])->getPosition1();
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
    // the position of the robot is the center of the base
    // to set it on the ground when the z component of the position is 0
    // base_length*0.5 is added (without this half of the base will be in the ground)    
    osg::Matrix p2;
    p2 = pose 
      // TODO: create is not robust enough to endure this pose !!
      //      * osg::Matrix::rotate(M_PI/2, 0, 0, 1) 
      * osg::Matrix::translate(osg::Vec3(0, 0, conf.base_length* 0.5)); 
    create(p2);


      // p->setPose(osg::Matrix::rotate(M_PI/2, 0, 0, 1) *
// 		 osg::Matrix::translate((n-half)*conf.segmLength, 0 , conf.segmDia/2) * 
// 		 pose);

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


  void Arm2Segm::doInternalStuff(const GlobalData& globalData){
  };

  void Arm2Segm::mycallback(void *data, dGeomID o1, dGeomID o2){
    // no internal collisions in this robot
  };

  bool Arm2Segm::collisionCallback(void *data, dGeomID o1, dGeomID o2){
    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space){
      
      // the rest is for collisions of some elements of the robot with the rest of the world
      int i,n;  
      const int N = 10;
      dContact contact[N];

      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++){
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
      return true;
    }
    return false;
  };
  

  Primitive* Arm2Segm::getMainPrimitive() const{ 
    // at the moment returning position of last arm,
    // better would be a tip at the end of this arm, as in muscledArm
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

    // create base
    Primitive* o = new Box(conf.base_length, conf.base_width, conf.base_length);
    o -> init(odeHandle, conf.base_mass, osgHandle);
    o->setPose( pose); // set base to given pose
    //o->getOSGPrimitive()->setTexture("Images/wood.rgb");
    objects.push_back(o);

    // create arms
    for (int i=0; i<(conf.segmentsno-1); i++){
      o = new Box(conf.arm_length, conf.arm_width, conf.arm_width);
      o -> init(odeHandle, conf.arm_mass, osgHandle);
      // move arms to desired places
      o -> setPose(osg::Matrix::translate(osg::Vec3(// shift (armlength -JOINT_OFFSET) to have overlapp
						    (i+0.5)*conf.arm_length - (i+1)*conf.joint_offset,
						    // shift to have place between arms
						    (i+1)*conf.arm_offset 
						    + (i+0.5)*conf.arm_width + 0.5*conf.base_width,
						    // height is ok, no shift
						    0) ) * pose);
      objects.push_back(o);
    }

    // hinge joint and angular motor to connect base with world
    Pos p1(objects[0]->getPosition());
    HingeJoint* j = new HingeJoint(0, objects[0], p1, osg::Vec3(0,0,1) /** pose*/);
    j -> init(odeHandle, osgHandle,/*withVisual*/true);
    joints.push_back(j);
    AngularMotor1Axis* a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[0], conf.max_force);
    amotors.push_back(a);

    // hinge joint and angular motor to connect base with first arm
    Pos p2(objects[1]->getPosition());
    p1[1]=(p1[1]+p2[1])/2;
    j = new HingeJoint(objects[0], objects[1], p1, osg::Vec3(0,1,0) /** pose*/);
    j -> init(odeHandle, osgHandle,/*withVisual*/true);
    joints.push_back(j);
    a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[1], conf.max_force);
    amotors.push_back(a);

    // hinge joint and angular motor to connect arms
    for (int i=2; i<conf.segmentsno; i++) {
      Pos po1(objects[i-1]->getPosition());
      Pos po2(objects[i]->getPosition());
      Pos po3( (po1+po2)/2);  
      j = new HingeJoint(objects[i-1], objects[i], po3, osg::Vec3(0,1,0) /** pose*/);
      j -> init(odeHandle, osgHandle,/*withVisual*/true);
      joints.push_back(j);
      a=new AngularMotor1Axis(odeHandle, (OneAxisJoint *)joints[i], conf.max_force);
      amotors.push_back(a);
    }

// --------------
// TODO: add tip at the end of arm for easier getPosition; temporarily positioning of transform object does not work
    osg::Matrix ps;
    ps.makeIdentity();
    Primitive* o1 = new Sphere(conf.arm_width*0.8);
    Primitive* o2 = new Transform(objects[objects.size()-1], o1, osg::Matrix::translate(0, conf.arm_length*0.5, 0) * ps);
    o2->init(odeHandle, /*mass*/0, osgHandle, /*withBody*/ false);
// --------------    

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
