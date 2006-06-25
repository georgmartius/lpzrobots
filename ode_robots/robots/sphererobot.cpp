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
 *   Revision 1.11.4.7  2006-06-25 16:57:16  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.11.4.6  2006/05/09 04:24:34  robot5
 *   *** empty log message ***
 *
 *   Revision 1.11.4.5  2006/01/10 17:17:17  martius
 *   new mode for primitives
 *
 *   Revision 1.11.4.4  2006/01/10 15:10:24  martius
 *   fine tuning, fuer controllinterval 1
 *   still not transparent
 *
 *   Revision 1.11.4.3  2005/12/30 22:51:45  martius
 *   moved to osg
 *
 *   Revision 1.11.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.11.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.11  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/
#include <assert.h>

#include "sphererobot.h"
#include "primitive.h"
#include "joint.h"
#include "sliderservo.h"
#include "invisibleprimitive.h"

#include "matrix.h"
using namespace matrix;

namespace lpzrobots {

  const int Sphererobot::sensorno;

  Sphererobot::Sphererobot ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			     const SphererobotConf& conf, const std::string& name )
    : OdeRobot ( odeHandle, osgHandle, name, "$ID$" ), conf(conf) {

    
    created = false;
    memset(object, 0,sizeof(void*) * Last);
    memset(joint,  0,sizeof(void*) * 6);
    memset(slider, 0,sizeof(void*) * 3);
  }
	
  Sphererobot::~Sphererobot() {
    destroy();
  }

  void Sphererobot::update(){
    assert(created); // robot must exist
  
    for (int i=0; i<Last; i++) { 
      if(object[i]) object[i]->update();
    }
    for (int i=0; i < 6; i++) { 
      if(joint[i]) joint[i]->update();
    }
    for (int i=0; i < 3; i++) { 
      if(slider[i]) slider[i]->update();
    }
  }
  

  int Sphererobot::getSensors ( sensor* sensors, int sensornumber ) {  
    int len = min(sensornumber, 3);
    for ( int n = 0; n < len; n++ ) {
      sensors[n] = servo[n]->get();
    }

    double data[3] = {1,0,0};
    Matrix v(3,1,data);
    Matrix A = odeRto3x3RotationMatrix(dBodyGetRotation(object[Base]->getBody()));
    Matrix v2 = A*v;
    v.val(0,0)=0;
    v.val(1,0)=1;
    Matrix v3 = A * v;
    int l= v2.convertToBuffer(sensors+3, sensornumber -3);
    return v3.convertToBuffer(sensors + l + 3 , sensornumber - l -3) + l + 3;
  }

  void Sphererobot::setMotors ( const motor* motors, int motornumber ) {
    int len = min(motornumber, 3);
    for ( int n = 0; n < len; n++ ) {
      servo[n]->set(motors[n]);
    }
  }


  void Sphererobot::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)    
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.diameter/2)); 
    create(p2);    
  };

  void Sphererobot::doInternalStuff(const GlobalData& global){}

  bool Sphererobot::collisionCallback(void *data, dGeomID o1, dGeomID o2) {
    //checks if both of the collision objects are part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space) {      
      int i,n;  
      const int N = 10;
      dContact contact[N];
    
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++) {
	if( contact[i].geom.g1 == object[Base]->getGeom() || contact[i].geom.g2 == object[Base]->getGeom() ){ 
	  // only treat collisions with envelop
	  contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
	  contact[i].surface.mu = 1.0;
	  contact[i].surface.soft_erp = 0.5;
	  contact[i].surface.soft_cfm = 0.1;
	  // 	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	  // 	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	  // 	contact[i].surface.mu = frictionGround;
	  // 	contact[i].surface.slip1 = 0.005;
	  // 	contact[i].surface.slip2 = 0.005;
	  // 	contact[i].surface.soft_erp = 1;
	  // 	contact[i].surface.soft_cfm = 0.00001;
	  dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
	} 
      }
      return true;
    } else {
      return false;
    }
  }


  /**
   *Returns the number of motors used by the snake.
   *@return number of motors
   *@author Marcel Kretschmann
   *@version final
   **/
  int Sphererobot::getMotorNumber(){
    return 3;
  }

  /**
   *Returns the number of sensors used by the robot.
   *@return number of sensors
   *@author Marcel Kretschmann
   *@version final
   **/
  int Sphererobot::getSensorNumber() {
    return sensorno;
  }

  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  int Sphererobot::getSegmentsPosition(vector<Position> &poslist){
    poslist.push_back(Pos(object[Base]->getPosition()).toPosition() );
    poslist.push_back(Pos(object[Pendular]->getPosition()).toPosition() );
    return 2;
  }


  
  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void Sphererobot::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    odeHandle.space = dSimpleSpaceCreate (parentspace);
    OsgHandle osgHandle_bottom = osgHandle.changeColor(Color(1.0, 0, 0));
    OsgHandle osgHandle_pendular = osgHandle.changeColor(Color(0.0, 1.0 , 0));
  
    object[Base] = new InvisibleSphere(conf.diameter/2);
    //object[Base] = new Sphere(conf.diameter/2);
    //object[Base] = new InvisibleBox(conf.diameter, conf.diameter, conf.diameter);
    object[Base]->init(odeHandle, conf.spheremass, osgHandle);
    object[Base]->setPose(pose);    

    //pendular body
    object[Pendular] = new Sphere(conf.pendulardiameter/2);
    object[Pendular]->init(odeHandle, conf.spheremass, osgHandle_pendular);
    object[Pendular]->setPose(pose);
              
    //first and second 3 connection bodies between the pendular an the sphere
    double x , y;
    for ( unsigned int alpha = 0; alpha < 3; alpha++ ) {
      x=sin ( (float) alpha*2*M_PI/3 )*conf.diameter/3.5; //testing values 
      y=cos ( (float) alpha*2*M_PI/3 )*conf.diameter/3.5;

      object[Pole1Bot+alpha] = new Box(conf.diameter/50, conf.diameter/50, conf.diameter/50);
      object[Pole1Bot+alpha]->init(odeHandle, conf.slidermass, osgHandle_bottom, 
				   Primitive::Body | Primitive::Draw);
      object[Pole1Bot+alpha]->setPose(osg::Matrix::translate(x,y,- conf.diameter/2 + conf.diameter/9) * 
				      pose);

      object[Pole1Top+alpha] = new Box(conf.diameter/50, conf.diameter/50, conf.diameter/50);
      object[Pole1Top+alpha]->init(odeHandle, conf.slidermass, osgHandle_pendular, 
				   Primitive::Body | Primitive::Draw);
      object[Pole1Top+alpha]->setPose(osg::Matrix::translate(x,y,0) * pose);

      //combines the 3 upper connection bodies with the pendular
      joint[alpha] = new HingeJoint(object[Pendular], object[Pole1Top+alpha], 
				    object[Pole1Top+alpha]->getPosition(),
				    osg::Vec3(y, -x, 0));
      joint[alpha]->init(odeHandle, osgHandle, true, conf.diameter/20);
      //  dJointSetHingeParam ( hinge, dParamLoStop, -conf.hingeRange);
      //     dJointSetHingeParam ( hinge, dParamHiStop,  conf.hingeRange);
      //     dJointSetHingeParam  ( hinge, dParamCFM, 0.1);
      //     dJointSetHingeParam ( hinge, dParamStopCFM, 0.1);
      //     dJointSetHingeParam ( hinge, dParamStopERP, 0.9);

      //combines the 3 lower connection bodies with the base
      joint[alpha+3] = new BallJoint(object[Base], object[Pole1Bot+alpha], 
				     object[Pole1Bot+alpha]->getPosition());
      joint[alpha+3]->init(odeHandle, osgHandle, true, conf.diameter/40) ;
      
      //definition of the 3 Slider-Joints, which are the controled by the robot-controler
      slider[alpha] = new SliderJoint(object[Pole1Top+alpha], object[Pole1Bot+alpha],
				      (object[Pole1Top+alpha]->getPosition() +
				       object[Pole1Bot+alpha]->getPosition())/2,
				      object[Pole1Top+alpha]->getPosition() -
				      object[Pole1Bot+alpha]->getPosition() );
      slider[alpha]->init(odeHandle, osgHandle, true, conf.diameter*conf.sliderrange);
      // the Stop parameters are messured from the initial position!
      slider[alpha]->setParam(dParamLoStop, -1.1*conf.diameter*conf.sliderrange );
      slider[alpha]->setParam(dParamHiStop, 1.1*conf.diameter*conf.sliderrange );
      slider[alpha]->setParam(dParamCFM, 0.1);
      slider[alpha]->setParam(dParamStopCFM, 0.1);
      slider[alpha]->setParam(dParamStopERP, 0.9);

      servo[alpha] = new SliderServo(slider[alpha], -conf.diameter*conf.sliderrange, 
 				     conf.diameter*conf.sliderrange, 
 				     conf.pendularmass*0.1 * conf.force);
  
    }

    created=true;
  }; 



  /** destroys vehicle and space
   */
  void Sphererobot::destroy(){
    if (created){
      for (int i=0; i<Last; i++){
	if(object[i]) delete object[i];
      }
      for (int i=0; i<6; i++){
	if(joint[i]) delete joint[i];
      }
      for (int i=0; i<3; i++){
	if(slider[i]) delete slider[i];
      }

      dSpaceDestroy(odeHandle.space);
    }
    created=false;
  }


}



