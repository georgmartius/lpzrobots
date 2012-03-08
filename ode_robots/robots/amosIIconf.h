/***************************************************************************
 *   Copyright (C) 2012 by                                                 *
 *    Martin Biehl <mab@physik3.gwdg.de>                                   *
 *    Guillaume de Chambrier <s0672742@sms.ed.ac.uk>                       *
 *    martius@informatik.uni-leipzig.de                                    *
 *    Timo Nachstedt <nachstedt@physik3.gwdg.de>                           *
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
 **************************************************************************/

#ifndef AMOSIICONF_H_
#define AMOSIICONF_H_

#include <selforg/matrix.h>
#include <iostream>

namespace lpzrobots{

struct AmosIIConf{
public:

    static AmosIIConf getDefaultConf(
            double _scale = 1.0,
            bool _useShoulder = 1,
            bool _useFoot = 1,
            bool _useBack = 0);

    /**
     * joint limits differ, density is reduced with scale
     */
    static AmosIIConf getExperimentConf(
            double _scale = 1.0,
            bool _useShoulder = 1,
            bool _useFoot = 1,
            bool _useBack = 0);

    static AmosIIConf getFixedConf(
            double _scale = 1.0,
            bool _useShoulder = 1,
            bool _useFoot = 1,
            bool _useBack = 0);

    static AmosIIConf getFranksDefaultConf(
            double _scale = 5.0,
            bool _useShoulder = 1,
            bool _useFoot = 1,
            bool _useBack = 0);


    /**
     * @name flags
     *
     * Enable or disable different element and features
     */
    /**@{*/
    /** fix the shoulder element to the trunk. */
    bool useShoulder;
    /** whether to use joints at the knees or fix them */
    bool useTebiaJoints;
    /** use spring foot */
    bool useFoot;
    /** use the hinge joint in the back */
    bool useBack;
    bool useWhiskers;
    /**
     * if true, rubber substance is used for feet instead of the substance used
     * for the rest of the robot
     */
    bool rubberFeet;
    /** use foot contact sensors */
    bool useContactSensors;
    /** get velocity vector in local coordinates and pass it as sensorvalues */
    bool useLocalVelSensor;
    /**@}*/


    /** scaling factor for robot (length of body) */
    double size;
    /** trunk width */
    double width;
    /** trunk height */
    double height;
    /** length of the front of the body (if back joint is used) */
    double frontLength;
    /** homogenous density used to scale the whole robot */
    double density;
    /** total mass */
    double mass;

    /** trunk mass */
    double trunkMass;
    double frontMass;
    double shoulderMass;
    double coxaMass;
    double secondMass;
    double tebiaMass;
    double footMass;

    /** fix legs to trunk at this distance from bottom of trunk */
    double shoulderHeight;

    /** distance between hindlegs and middle legs */
    double legdist1;

    /** distance between middle legs and front legs */
    double legdist2;


    /** @name Leg extension from trunk
     *
     *  amosII has a fixed but adjustable joint that decides how the legs extend
     *  from the trunk.here you can adjust these joints, if shoulder = 0 this
     *  still influences how the legs extend (the coxa-trunk connection in that
     *  case)
     */
    /**@{*/
    /** angle in rad around vertical axis at leg-trunk fixation 0: perpendicular
     */
    double fLegTrunkAngleV;
    /** angle around horizontal axis at leg-trunk fixation 0: perpendicular */
    double fLegTrunkAngleH;
    /** rotation of leg around own axis 0: first joint axis is vertical */
    double fLegRotAngle;
    /** middle legs and so on */
    double mLegTrunkAngleV;
    double mLegTrunkAngleH;
    double mLegRotAngle;
    double rLegTrunkAngleV;
    double rLegTrunkAngleH;
    double rLegRotAngle;
    /**@}*/

    /**
     * @name zero position of joints
     *
     * the zero position of the joints depends on the placement of the two
     * bodies at the time of initialisation. specify in the following at which
     * angle the leg part should extend from the previous leg part (in rad).
     * this sets the 0 angle!
     */
    /**@{*/
    /** front, negative is forward */
    double fcoxaZero;
    /** positive is down */
    double fsecondZero;
    /** positive is down */
    double ftebiaZero;
    /**@}*/


    /**
     * @name leg part dimensions
     *
     * the lengths and radii of the individual leg parts
     */
    /**@{*/
    double legLength;
    double shoulderLength;
    double shoulderRadius;
    double coxaLength;
    double coxaRadius;
    double secondLength;
    double secondRadius;
    /** length of tebia including  fully extended foot spring (if used) */
    double tebiaLength;
    double tebiaRadius;
    /** range of the "foot" spring */
    double footRange;
    /** choose different from tebiaRadius */
    double footRadius;
    /**@}*/


    /**
     * @name Joint Limits
     *
     * set limits for each joint
     */
    /**{*/
    /** smaller limit, positive is down */
    double backJointLimitD;
    double backJointLimitU;
    /** forward limit, negative is forward (zero was specified above) */
    double fcoxaJointLimitF;
    /** backward limit */
    double fcoxaJointLimitB;
    /** forward limit, negative is forward (zero was specified above) */
    double mcoxaJointLimitF;
    /** backward limit */
    double mcoxaJointLimitB;
    /** forward limit, negative is forward (zero was specified above) */
    double rcoxaJointLimitF;
    /** backward limit */
    double rcoxaJointLimitB;
    /** lower limit, positive is down */
    double secondJointLimitD;
    /** upper limit */
    double secondJointLimitU;
    /** lower limit, positive is down */
    double tebiaJointLimitD;
    double tebiaJointLimitU;
    /**}*/


    /** preload spring */
    double footSpringPreload;
    /** upper limit negative is up */
    double footSpringLimitU;
    double footSpringLimitD;


    double backPower;
    /** maximal force for at hip joint motors */
    double coxaPower;
    double secondPower;
    /** spring strength in the knees */
    double tebiaPower;
    double footPower;

    double backDamping;
    /** damping of hip joint servos */
    double coxaDamping;
    double secondDamping;
    /** damping in the knees */
    double tebiaDamping;
    double footDamping;

    double backMaxVel;
    /** speed of the hip servo */
    double coxaMaxVel;
    double secondMaxVel;
    double tebiaMaxVel;
    double footMaxVel;


    // length of whisker (currently not implemented
    double whiskerLength;

    /**
     * configure the front ultrasonic sensors
     */
    /** angle versus x axis */
    double usAngleX;
    /** angle versus y axis */
    double usAngleY;
    /** choose between parallel or antiparallel front ultrasonic sensors true
     *  means parallel */
    bool usParallel;
    /** range of the front ultrasonic sensors */
    double usRangeFront;
    /** range of the infrared sensors at the legs */
    double irRangeLeg;

    // path to texture for trunks and legs
    std::string texture;
  };

}
#endif /* AmosIIConf_H_ */
