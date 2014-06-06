
/***************************************************************************
 *   Copyright (C) 2005-2012 LpzRobots development team                    *
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
#ifndef __ROBOTCHAIN_H
#define __ROBOTCHAIN_H

#include <ode_robots/oderobot.h>

namespace lpzrobots {

  typedef struct {
  public:
    double      size;           ///< scaling factor for robot (size of one robot)
    double      distance;       ///< distance between robots
    double      force;          ///< @see Nimm2Conf
    double      speed;          ///< @see Nimm2Conf
    int         numRobots;      ///< number of robots in the chain
    double      massFactor;     ///< @see Nimm2Conf
    double      wheelSlip;      ///< @see Nimm2Conf
    std::string color;          ///< color of robots
    bool        useIR;          ///< use infrared sensors?
    int         mainRobot;      ///< robot index that is returned by getMainPrimitive() (-1 for middle robot, then massFactorMain and forceMain is ignored)
    double      massFactorMain; ///< massfactor of main robot @see Nimm2Conf
    double      forceMain;      ///< factor of main robot @see Nimm2Conf
  } RobotChainConf;


  /** Chain of robots
   */
  class RobotChain : public OdeRobot {
  public:

    /**
     * constructor of uwo robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param size scaling of robot
     * @param force maximal used force to realize motorcommand
     * @param radialLegs switches between cartensian and radial leg joints
     */
    RobotChain(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
               const RobotChainConf& conf, const std::string& name);

    virtual ~RobotChain(){ destroy(); };

    static RobotChainConf getDefaultConf(){
      RobotChainConf c;
      c.numRobots  = 5;
      c.size       = 0.6;
      c.distance   = 0.95;
      c.force      = 2;
      c.speed      = 45;
      c.massFactor = 0.5;
      c.wheelSlip  = 0.02;
      c.color      = "robot3";
      c.useIR      = false;
      c.mainRobot  = -1;
      c.massFactorMain  = c.massFactor;
      c.forceMain  = c.force;
      return c;
    }

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update();


    /** sets the pose of the vehicle
        @param pose desired pose matrix
    */
    virtual void placeIntern(const osg::Matrix& pose);

    virtual int getSensorsIntern(sensor* sensors, int sensornumber);

    virtual void setMotorsIntern(const double* motors, int motornumber);

    virtual int getSensorNumberIntern();

    virtual int getMotorNumberIntern();

    virtual void doInternalStuff(GlobalData& globalData);

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);


    virtual int getIRSensorNum();

  protected:
    virtual Primitive* getMainPrimitive() const;

    virtual void create(const osg::Matrix& pose);

    virtual void destroy();

    RobotChainConf conf;

    bool created;

    std::vector <OdeRobot*> robots;
  };

}

#endif
