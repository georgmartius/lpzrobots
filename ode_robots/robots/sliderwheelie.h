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
 *   Revision 1.15  2010-01-27 10:19:14  martius
 *   showCenter variable added
 *
 *   Revision 1.14  2010/01/26 09:56:31  martius
 *   added dummy center also with velocity
 *
 *   Revision 1.13  2009/08/10 07:48:55  guettler
 *   removed typedef to avoid compiler warnings
 *
 *   Revision 1.12  2009/03/27 20:45:03  martius
 *   motor type can be selected
 *
 *   Revision 1.11  2009/03/26 18:01:59  martius
 *   angular motors possible
 *   sliders can be switched off -> defaultwheelie is obsolete
 *   better drawing of joints
 *   all motors are set (was a bug before)
 *
 *   Revision 1.10  2008/09/16 14:53:24  martius
 *   provide a virtual center of the robot as main primitive
 *
 *   Revision 1.9  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.8  2007/01/26 12:05:04  martius
 *   servos combinied into OneAxisServo
 *
 *   Revision 1.7  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.6  2006/09/21 22:09:58  martius
 *   collision for mesh
 *
 *   Revision 1.5  2006/09/21 16:17:18  der
 *   *** empty log message ***
 *
 *   Revision 1.4  2006/09/21 08:15:15  martius
 *   with sliders inside a segment
 *
 *   Revision 1.3  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/06/25 21:57:20  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.2  2006/06/20 07:18:29  robot3
 *   -added cvs log
 *   -changed some behaviour of wheelie
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __SLIDERWHEELIE_H
#define __SLIDERWHEELIE_H

#include<vector>
#include<assert.h>

#include "oderobot.h"
#include"primitive.h"
#include "joint.h"
#include "angularmotor.h"

#include "oneaxisservo.h"


namespace lpzrobots {

  typedef struct {
  public:
    /* typedef */ enum MotorType {Servo, CenteredServo, AngularMotor };

    int    segmNumber;    ///<  number of snake elements
    double segmLength;    ///< length of one snake element
    double segmDia;       ///<  diameter of a snake element
    double segmMass;      ///<  mass of one snake element
    double motorPower;    ///<  power of the motors / servos
    double motorDamp;     ///<  damping of motors
    double powerRatio;    ///< ratio of motorpower for hinge vs. slider
    double sensorFactor;  ///<  scale for sensors
    double frictionGround;///< friction with ground
    double frictionJoint; ///< friction within joint
    double jointLimitIn;  ///< maximal angle for the joints to the inside (M_PI/2 = 90 degree)
    double jointLimitOut; ///< maximal angle for the joints to the outside
    double sliderLength;  ///< length of the slider in segmLength (0 for no sliders)
    MotorType motorType;  ///< whether to use servos or angular motors
    bool   showCenter;    ///< whether to show the virtual center
  } SliderWheelieConf;
  

  /**
   * This is a class, which models an annular robot. 
   * It consists of a number of equal elements, each linked 
   * by a joint powered by 1 servo
   **/
  class SliderWheelie : public OdeRobot
  {
  private:
    bool created;
      
    
    std::vector <Primitive*> objects;
    std::vector <Joint*> joints;
    std::vector <AngularMotor*> angularMotors;
    SliderWheelieConf conf;

    std::vector <HingeServo*> hingeServos;
    std::vector <SliderServo*> sliderServos;

    Primitive* center; // virtual center object (position updated on setMotors)
    DummyPrimitive* dummycenter; // virtual center object (here we can also update velocity)
  public:
    SliderWheelie(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		  const SliderWheelieConf& conf, const std::string& name, 
		  const std::string& revision = "");
    
    virtual ~SliderWheelie();
	
    static SliderWheelieConf getDefaultConf(){
      SliderWheelieConf conf;
      conf.segmNumber = 8;       //  number of snake elements
      conf.segmLength = 0.4;     // length of one snake element
      conf.segmDia    = 0.2;     //  diameter of a snake element
      conf.segmMass   = 0.4;     //  mass of one snake element
      conf.motorPower = 5;       //  power of the servos
      conf.motorDamp  = 0.01;    //  damping of servos
      conf.powerRatio = 2;       //  power of the servos
      conf.sensorFactor    = 1;   //  scale for sensors
      conf.frictionGround  = 0.8; // friction with ground
      conf.frictionJoint   = 0.0; // friction within joint
      conf.jointLimitIn    =  M_PI/3;
      conf.jointLimitOut   =  -1; // automatically set to 2*M_PI/segm_num
      conf.sliderLength    =  1;  
      conf.motorType       = SliderWheelieConf::CenteredServo; // use centered servos
<<<<<<< sliderwheelie.h
      conf.showCenter      = false;
=======
      conf.drawCenter      = true;
>>>>>>> 1.14
      return conf;
    }

    virtual void place(const osg::Matrix& pose);
    
    virtual void update();

    void doInternalStuff(GlobalData& global);

    bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

    virtual void setMotors ( const motor* motors, int motornumber );

    virtual int getSensors ( sensor* sensors, int sensornumber );
	
    virtual int getSensorNumber() { assert(created); 
      return hingeServos.size()+angularMotors.size()+sliderServos.size(); }

    virtual int getMotorNumber(){ assert(created); 
      return hingeServos.size()+angularMotors.size()+sliderServos.size(); }

    virtual Primitive* getMainPrimitive() const {
      if(dummycenter) return dummycenter;
      else if(!objects.empty()){
	return (objects[0]);
      }else return 0;
    } 
    
    virtual paramlist getParamList() const;
    
    virtual paramval getParam(const paramkey& key) const;
    
    virtual bool setParam(const paramkey& key, paramval val);

  private:
    static void mycallback(void *data, dGeomID o1, dGeomID o2);

    virtual void create(const osg::Matrix& pose);
    virtual void destroy();
  };

}

#endif
