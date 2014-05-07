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

    std::string texture;  ///< texture for segments
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
      conf.showCenter      = false;
      conf.texture         = "";
      return conf;
    }

    virtual void placeIntern(const osg::Matrix& pose);

    virtual void update();

    void doInternalStuff(GlobalData& global);

    virtual void setMotorsIntern( const double* motors, int motornumber );

    virtual int getSensorsIntern( sensor* sensors, int sensornumber );

    virtual int getSensorNumberIntern() { assert(created);
      return hingeServos.size()+angularMotors.size()+sliderServos.size(); }

    virtual int getMotorNumberIntern(){ assert(created);
      return hingeServos.size()+angularMotors.size()+sliderServos.size(); }

    virtual Primitive* getMainPrimitive() const {
      if(dummycenter) return dummycenter;
      else if(!objects.empty()){
        return (objects[0]);
      }else return 0;
    }

    virtual std::vector<Primitive*> getAllPrimitives() const { return objects;}

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);

  private:
    static void mycallback(void *data, dGeomID o1, dGeomID o2);

    virtual void create(const osg::Matrix& pose);
    virtual void destroy();
  };

}

#endif
