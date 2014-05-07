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
#include <assert.h>

#include "sphererobot.h"
#include "primitive.h"
#include "joint.h"
#include "oneaxisservo.h"
#include "mathutils.h"

#include <selforg/matrix.h>
using namespace matrix;
using namespace std;

namespace lpzrobots {

  const int Sphererobot::sensorno;

  Sphererobot::Sphererobot ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                             const SphererobotConf& conf, const std::string& name )
    : OdeRobot ( odeHandle, osgHandle, name, "$Id$" ), conf(conf) {


    created = false;
    memset(object, 0,sizeof(void*) * Last);
    memset(joint,  0,sizeof(void*) * 6);
    memset(slider, 0,sizeof(void*) * 3);
  }

  Sphererobot::~Sphererobot() {
    destroy();
  }

  void Sphererobot::update() {
    OdeRobot::update();
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


  int Sphererobot::getSensorsIntern( double* sensors, int sensornumber ) {
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

  void Sphererobot::setMotorsIntern( const double* motors, int motornumber ) {
    int len = min(motornumber, 3);
    for ( int n = 0; n < len; n++ ) {
      servo[n]->set(motors[n]);
    }
  }


  void Sphererobot::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.diameter/2));
    create(p2);
  };


  /**
   *Returns the number of motors used by the snake.
   *@return number of motors
   *@author Marcel Kretschmann
   *@version final
   **/
  int Sphererobot::getMotorNumberIntern(){
    return 3;
  }

  /**
   *Returns the number of sensors used by the robot.
   *@return number of sensors
   *@author Marcel Kretschmann
   *@version final
   **/
  int Sphererobot::getSensorNumberIntern() {
    return sensorno;
  }

  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments)
      @return length of the list
  */
  int Sphererobot::getSegmentsPosition(std::vector<Position> &poslist){
    poslist.push_back(Pos(object[Base]->getPosition()).toPosition() );
    poslist.push_back(Pos(object[Pendular]->getPosition()).toPosition() );
    return 2;
  }



  /** creates vehicle at desired position
  */
  void Sphererobot::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    odeHandle.createNewSimpleSpace(parentspace,true);
    OsgHandle osgHandle_bottom = osgHandle.changeColor(Color(1.0, 0, 0));
    OsgHandle osgHandle_pendular = osgHandle.changeColor(Color(0.0, 1.0 , 0));

    object[Base] = new Sphere(conf.diameter/2);
    //object[Base] = new Sphere(conf.diameter/2);
    //object[Base] = new InvisibleBox(conf.diameter, conf.diameter, conf.diameter);
    //    object[Base]->init(odeHandle, conf.spheremass, osgHandle, Primitive::Body | Primitive::Geom);
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
      for (int i=0; i<3; i++){
        if(slider[i]) delete slider[i];
      }
      for (int i=0; i<6; i++){
        if(joint[i]) delete joint[i];
      }
      for (int i=0; i<Last; i++){
        if(object[i]) delete object[i];
      }

      odeHandle.deleteSpace();
    }
    created=false;
  }


}



