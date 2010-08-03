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
 *   Revision 1.22  2010-08-03 12:50:39  martius
 *   Servo interface changes: damping() is now get/setDamping
 *
 *   Revision 1.21  2010/01/27 10:19:11  martius
 *   showCenter variable added
 *
 *   Revision 1.20  2010/01/26 09:56:31  martius
 *   added dummy center also with velocity
 *
 *   Revision 1.19  2009/03/27 20:45:03  martius
 *   motor type can be selected
 *
 *   Revision 1.18  2009/03/26 20:25:35  martius
 *   changed color to gold; segments are equally wide
 *
 *   Revision 1.17  2009/03/26 18:01:59  martius
 *   angular motors possible
 *   sliders can be switched off -> defaultwheelie is obsolete
 *   better drawing of joints
 *   all motors are set (was a bug before)
 *
 *   Revision 1.16  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.15  2008/09/16 14:52:58  martius
 *   provide a virtual center of the robot as main primitve
 *
 *   Revision 1.14  2008/05/07 16:45:52  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.13  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.12  2007/09/06 18:48:00  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.11  2007/07/03 13:05:23  martius
 *   new servo constants
 *
 *   Revision 1.10  2007/03/30 17:52:28  martius
 *   sensor and motor assignment fixed
 *   friction with ground adaptable
 *
 *   Revision 1.9  2007/03/16 10:58:37  martius
 *   new collision control via substances
 *   fixed Bug in creation
 *
 *   Revision 1.8  2006/09/21 11:43:45  martius
 *   powerratio
 *
 *   Revision 1.7  2006/09/21 10:21:33  robot8
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
	center=0;
	dummycenter=0;
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
    if(center) center->update();      
  }

  void SliderWheelie::doInternalStuff(GlobalData& global){
  }

  void SliderWheelie::setMotors(const motor* motors, int motornumber) {
   assert(created);
   unsigned int len = min(motornumber, getMotorNumber());
   unsigned int n=0;
   // controller output as torques 
   if(conf.motorType != SliderWheelieConf::AngularMotor){
     for(unsigned int i=0; (n<len) && (i<hingeServos.size()); i++, n++) {
       hingeServos[i]->set(motors[n]);
     }
   }else{
     for(unsigned int i=0; (n<len) && (i<angularMotors.size()); i++, n++) {
       angularMotors[i]->set(1, motors[n]);
     }
   }

   for(unsigned int i=0; (n<len) && (i<sliderServos.size()); i++, n++) {
     sliderServos[i]->set(motors[n]);
   }
   
   /// update center position
   if(center){ 
     Pos p;
     Pos v;
     for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
       p += (*i)->getPosition();
       v += (*i)->getVel();
     }
     p /= objects.size();
     v /= objects.size();
     center->setPosition(p);
     dummycenter->setPosition(p);
     dummycenter->setVel(v);
   }
  }

  int SliderWheelie::getSensors(sensor* sensors, int sensornumber) {
   assert(created);
   unsigned int len=min(sensornumber, getSensorNumber());
   unsigned int n=0;
   // get the hingeServos
   if(conf.motorType != SliderWheelieConf::AngularMotor){
     for(unsigned int i=0; (n<len) && (i<hingeServos.size()); i++, n++) {
       sensors[n] = hingeServos[i]->get();
     }
   }else{
     for(unsigned int i=0; (n<len) && (i<angularMotors.size()); i++, n++) {
       sensors[n] = angularMotors[i]->get(1);
     }
   }

   for(unsigned int i=0; (n<len) && (i<sliderServos.size()); i++,n++) {
     sensors[n] = sliderServos[i]->get();
   }
   return n;
  }

  bool SliderWheelie::collisionCallback(void *data, dGeomID o1, dGeomID o2)
  {
    return false;
  }

  void SliderWheelie::create(const osg::Matrix& pose) {
    if (created) {
      destroy();
    }
    
    odeHandle.createNewSimpleSpace(parentspace,false);
    //    odeHandle.substance.toRubber(10);
	
    if(conf.jointLimitOut<0){
      conf.jointLimitOut = 2*M_PI/conf.segmNumber;
    }
    vector<Pos> ancors;
    // angular positioning
    for(int n = 0; n < conf.segmNumber; n++) {
      osg::Matrix m = osg::Matrix::rotate(M_PI*2*n/conf.segmNumber, 0, -1, 0) * pose;
      if(n%2==0 && conf.sliderLength > 0){ // slider segment
	
	Primitive* p1 = new Box(conf.segmDia/2, conf.segmDia*4*2, conf.segmLength/2);
	// p1->setTexture("Images/wood.rgb");
	p1->init(odeHandle, conf.segmMass/2 , osgHandle);
	p1->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
		    osg::Matrix::translate(-conf.segmLength/4,0,
					   -0.5*conf.segmLength*conf.segmNumber/M_PI) * m );	
	objects.push_back(p1);

	Primitive* p2 = new Box(conf.segmDia/2, conf.segmDia*4*2, conf.segmLength/2);
	// p2->setTexture("Images/dusty.rgb");
	p2->init(odeHandle, conf.segmMass/2 , osgHandle);
	p2->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
		    osg::Matrix::translate(conf.segmLength/4,0,
					   -0.5*conf.segmLength*conf.segmNumber/M_PI) * m );		
	objects.push_back(p2);

	const Pos& pos1(p1->getPosition());
	const Pos& pos2(p2->getPosition());
	
	SliderJoint* j = new SliderJoint(p1, p2, (pos1 + pos2)/2,
					 Axis(1,0,0)*m);
	j->init(odeHandle, osgHandle, true, conf.segmDia);
	
	joints.push_back(j);
	
	SliderServo* servo = new SliderServo(j, 
					     -conf.segmLength*conf.sliderLength/2, 
					     conf.segmLength*conf.sliderLength/2, 
					     conf.motorPower*conf.powerRatio);
	sliderServos.push_back(servo);	
      }else{ // normal segment
	Primitive* p1 = new Box(conf.segmDia/2, 
				/* conf.segmDia*4*( (n+1)%4 ==0 ? 3 : (n%2 ==0 ? 2 : 1)), */
				conf.segmDia*8,
				conf.segmLength-conf.segmDia/2);
	// p1->setTexture("Images/whitemetal_farbig_small.rgb");
	p1->init(odeHandle, conf.segmMass * ( (n+1)%4 ==0 ? 1.0 : 1), osgHandle);
	p1->setPose(osg::Matrix::rotate(M_PI*0.5, 0, 1, 0) *
		    osg::Matrix::translate(0,0,-0.5*conf.segmLength*conf.segmNumber/M_PI) * m );	
	objects.push_back(p1);
      }
      ancors.push_back(Pos(conf.segmLength/2,0,-0.5*conf.segmLength*conf.segmNumber/M_PI) * m);
     }


   //***************** hinge joint definition***********
    int i = 0;
    for(int n=0; n < conf.segmNumber ; n++, i++) {            
      if(n%2==0 && conf.sliderLength > 0){
	i++;	
      }
      int o1 = i;
      int o2 = (i+1) % objects.size();
      HingeJoint* j = new HingeJoint(objects[o1], objects[o2],
				     ancors[n],
				     Axis(0,1,0)*pose);
      Color c;
      if(n==0) c = Color(0.85,0.88,0.88);
      else if(n == conf.segmNumber/2) c = Color(0.6,0.56,0); 
      else c = osgHandle.color;
      j->init(odeHandle, osgHandle.changeColor(c), true, conf.segmDia*4);      
      joints.push_back(j);
      
      HingeServo* servo;
      switch (conf.motorType){	
      case SliderWheelieConf::Servo: 
	servo = new HingeServo(j, -conf.jointLimitOut, 
					   conf.jointLimitIn, 
					   conf.motorPower, conf.motorDamp,0);
	hingeServos.push_back(servo);
	break;
      case SliderWheelieConf::CenteredServo: 
	servo = new OneAxisServoCentered(j, -conf.jointLimitOut, conf.jointLimitIn,
					 conf.motorPower, conf.motorDamp,0);
	hingeServos.push_back(servo);
	break;
      case SliderWheelieConf::AngularMotor:  
	AngularMotor* amotor = new AngularMotor1Axis(odeHandle, j, conf.motorPower);
	j->setParam(dParamLoStop, -conf.jointLimitOut);
	j->setParam(dParamHiStop, conf.jointLimitIn);
	angularMotors.push_back(amotor);
	break;
      }
    }

    // create virtual center
    center = new Sphere(0.1);
    OdeHandle centerHandle = odeHandle;
    centerHandle.substance.toNoContact();    
    center->init(centerHandle, 0, osgHandle.changeAlpha(0.4), 
                 conf.showCenter ? Primitive::Geom | Primitive::Draw : Primitive::Geom); 
    center->setPose(osg::Matrix::translate(0,0,0) * pose);
    dummycenter = new DummyPrimitive();
    dummycenter->setPosition(center->getPosition());
    
    created=true;

  }

  /** destroys vehicle and space
   */
  void SliderWheelie::destroy() {
    if(created) {
      FOREACH(vector<AngularMotor*>, angularMotors, i){
	if(*i) delete *i;
      }
      angularMotors.clear();

      FOREACH(vector<HingeServo*>, hingeServos, i){
	if(*i) delete *i;
      }
      hingeServos.clear();

      FOREACH(vector<SliderServo*>, sliderServos, i){
	if(*i) delete *i;
      }
      sliderServos.clear();

      FOREACH(vector<Joint*>, joints, i){
	if(*i) delete *i;
      }
      joints.clear();
      FOREACH(vector<Primitive*>, objects, i){
	if(*i) delete *i;
      }
      if(center) delete center;
      if(dummycenter) delete dummycenter;
      objects.clear();
      odeHandle.deleteSpace();
   }
  }

  /** The list of all parameters with there value as allocated lists.
      @return list of parameters
  */
  Configurable::paramlist SliderWheelie::getParamList() const{
    paramlist list;
    list += pair<paramkey, paramval> (string("frictionground"), conf.frictionGround);
    list += pair<paramkey, paramval> (string("powerratio"), conf.powerRatio);
    list += pair<paramkey, paramval> (string("motorpower"),   conf.motorPower);
    list += pair<paramkey, paramval> (string("motordamp"),   conf.motorDamp);
    list += pair<paramkey, paramval> (string("sensorfactor"), conf.sensorFactor);
    return list;
  }
  
  
  Configurable::paramval SliderWheelie::getParam(const paramkey& key) const{    
    if(key == "frictionground") return conf.frictionGround; 
    else if(key == "powerratio") return conf.powerRatio; 
    else if(key == "motorpower") return conf.motorPower; 
    else if(key == "motordamp") return conf.motorDamp; 
    else if(key == "sensorfactor") return conf.sensorFactor; 
    else  return Configurable::getParam(key) ;
  }
  
  bool SliderWheelie::setParam(const paramkey& key, paramval val){    
    if(key == "frictionground") {
      conf.frictionGround = val; 
      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++) {
	if(*i) (*i)->substance.roughness=val;
      }      
    } else if(key == "motorpower") { 
      conf.motorPower = val; 

      FOREACH(vector<HingeServo*>, hingeServos, i) {
	if(*i) (*i)->setPower(conf.motorPower);
      }      
      FOREACH(vector<AngularMotor*>, angularMotors, i) {
	if(*i) (*i)->setPower(conf.motorPower);
      }      
    } else if(key == "motordamp") { 
      conf.motorDamp = val; 

      FOREACH(vector<HingeServo*>, hingeServos, i) {
	if(*i) (*i)->setDamping(conf.motorDamp);
      }
    }
    else if(key == "sensorfactor") conf.sensorFactor = val; 
    else if(key == "powerratio") { 
      conf.powerRatio = val; 
      for(vector<SliderServo*>::iterator i=sliderServos.begin(); i!=sliderServos.end(); i++) {
	if(*i) (*i)->setPower(conf.motorPower * conf.powerRatio);
      }      
    } else 
      return Configurable::setParam(key, val);    
    return true;
  }


}
