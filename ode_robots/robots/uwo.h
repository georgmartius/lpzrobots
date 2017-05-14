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
#ifndef __UWO_H
#define __UWO_H

#include "oderobot.h"
#include "oneaxisservo.h"
#include "twoaxisservo.h"

namespace lpzrobots {

  class Primitive;
  class Joint;

  typedef struct {
  public:
    double size;       ///< scaling factor for robot (diameter of body)
    double legLength;  ///< length of the legs in units of size
    int    legNumber;  ///<  number of snake elements
    bool   radialLegs; ///< joint orientation is radial instead of cartesian
    bool   useSliders; ///< use sliders at legs
    double mass;       ///< chassis mass
    double relLegmass; ///< relative overall leg mass
    double jointLimit; ///< angle range for legs
    double sliderLength;///< length of sliders at legs
    double motorPower; ///< maximal force for motors
    double sliderPowerFactor; ///< power factor for slider motors
  } UwoConf;


  /** UWO: Unknown Walk Object :-), looks like a plate with a lot of legs
   */
  class Uwo : public OdeRobot {
  public:

    /**
     * constructor of uwo robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param size scaling of robot
     * @param force maximal used force to realize motorcommand
     * @param radialLegs switches between cartensian and radial leg joints
     */
    Uwo(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const UwoConf& conf,
        const std::string& name);

    virtual ~Uwo(){ destroy(); };

    static UwoConf getDefaultConf(){
      UwoConf c;
      c.size       = 1;
      c.legNumber  = 8;
      c.legLength  = 0.3;
      c.mass       = 1;
      c.useSliders = true;
      c.relLegmass = 1;
      c.motorPower = 1;
      c.sliderPowerFactor = 3;
      c.jointLimit = M_PI/12; // +- 15 degree
      c.sliderLength=c.legLength/2;
      c.radialLegs = true;
      return c;
    }

    /** sets the pose of the vehicle
        @param pose desired pose matrix
    */
    virtual void placeIntern(const osg::Matrix& pose);


    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);

  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return objects[0]; }

    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();

    UwoConf conf;
    double legmass;    // leg mass

    bool created;      // true if robot was created

    std::vector <std::shared_ptr<TwoAxisServo> > servos;
    std::vector <std::shared_ptr<OneAxisServo> > sliderservos;
  };

}

#endif
