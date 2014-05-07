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
#ifndef __FOUR_WHEELED__
#define __FOUR_WHEELED__

#include "nimm4.h"
#include "raysensorbank.h"

namespace lpzrobots {

  class Primitive;
  class Hinge2Joint;
  class Joint;

  typedef struct {
    double size;
    double force;
    double speed;
    bool sphereWheels;
    bool useBumper;
    bool useButton; ///< use yellow Button at the back
    bool twoWheelMode; ///< if true then the robot emulates 2 wheels
    bool irFront;
    bool irBack;
    bool irSide;
    double irRangeFront;
    double irRangeBack;
    double irRangeSide;
    Substance wheelSubstance;
  } FourWheeledConf;

  /** Robot is based on nimm4 with
      4 wheels and a capsule like body
  */
  class FourWheeled : public Nimm4{
  public:

    /**
     * constructor of nimm4 robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration structure
     * @param name name of the robot
     */
    FourWheeled(const OdeHandle& odeHandle, const OsgHandle& osgHandle, FourWheeledConf conf, const std::string& name);

    static FourWheeledConf getDefaultConf(){
      FourWheeledConf conf;
      conf.size         = 1;
      conf.force        = 3;
      conf.speed        = 15;
      conf.sphereWheels = true;
      conf.twoWheelMode = false;
      conf.useBumper    = true;
      conf.useButton    = false;
      conf.irFront      = false;
      conf.irBack       = false;
      conf.irSide       = false;
      conf.irRangeFront = 3;
      conf.irRangeSide  = 2;
      conf.irRangeBack  = 2;
      conf.wheelSubstance.toRubber(40);
      return conf;
    }

    virtual ~FourWheeled();

    virtual int getSensorNumberIntern();
    virtual int getMotorNumberIntern();

    virtual int getSensorsIntern(sensor* sensors, int sensornumber);

    virtual void setMotorsIntern(const double* motors, int motornumber);

    // returns the joint with index i
    virtual Joint* getJoint(int i);

  protected:
    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();

    FourWheeledConf conf;
    Primitive* bumpertrans;
    Primitive* bumper;
  };

}

#endif
