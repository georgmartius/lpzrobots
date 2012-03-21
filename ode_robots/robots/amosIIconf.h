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
    /**
     * if true, rubber substance is used for feet instead of the substance used
     * for the rest of the robot
     */
    bool rubberFeet;
    /** decide whether you wand to use a local velocity sensors.
     *  If yes it gets velocity vector in local coordinates and pass it as
     *  sensorvalues */
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

    /** trunk mass */
    double trunkMass;
    /** mass of the front part of the robot (if backboine joint is used) */
    double frontMass;
    /** mass of the shoulders (if used) */
    double shoulderMass;
    /** mass of the coxa limbs */
    double coxaMass;
    /** mass of the second limbs */
    double secondMass;
    /** mass of the tebia limbs */
    double tebiaMass;
    /** mass of the feet */
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
    /** angle in rad around vertical axis at leg-trunk fixation for front legs*/
    double fLegTrunkAngleV;
    /** angle in rad around horizontal axis at leg-trunk fixation for front legs
     * */
    double fLegTrunkAngleH;
    /** rotation of front legs around own axis */
    double fLegRotAngle;
    /** angle in rad around vertical axis at leg-trunk fixation for middle legs
     * */
    double mLegTrunkAngleV;
    /** angle in rad around horizontal axis at leg-trunk fixation for middle
     * legs */
    double mLegTrunkAngleH;
    /** rotation of middle legs around own axis */
    double mLegRotAngle;
    /** angle in rad around vertical axis at leg-trunk fixation for rear legs*/
    double rLegTrunkAngleV;
    /** angle in rad around horizontal axis at leg-trunk fixation for rear legs
     * */
    double rLegTrunkAngleH;
    /** rotation of rear legs around own axis */
    double rLegRotAngle;
    /**@}*/

    /**
     * @name leg part dimensions
     *
     * the lengths and radii of the individual leg parts
     */
    /**@{*/
    /** length of the shoulder limbs (if used) */
    double shoulderLength;
    /** radius of the shoulder limbs (if used) */
    double shoulderRadius;
    /** length of the coxa limbs */
    double coxaLength;
    /** radius of the coxa limbs */
    double coxaRadius;
    /** length of the second limbs */
    double secondLength;
    /** radius of the second limbs */
    double secondRadius;
    /** length of the tebia limbs including fully extended foot spring
     *  (if used) */
    double tebiaLength;
    /** radius of the tebia limbs */
    double tebiaRadius;
    /** range of the foot spring */
    double footRange;
    /** radius of the foot capsules, choose different from tebiaRadius */
    double footRadius;
    /**@}*/


    /**
     * @name Joint Limits
     *
     * set limits for each joint
     */
    /**{*/
    /** smaller limit of the backbone joint, positive is down */
    double backJointLimitD;
    /** upper limit of the backbone joint, positive is down */
    double backJointLimitU;
    /** forward limit of the front TC joints, negative is forward
     *  (zero specified by fcoxazero) */
    double fcoxaJointLimitF;
    /** backward limit of the front TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double fcoxaJointLimitB;
    /** forward limit of the middle TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double mcoxaJointLimitF;
    /** backward limit of the middle TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double mcoxaJointLimitB;
    /** forward limit of the rear TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double rcoxaJointLimitF;
    /** backward limit of the rear TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double rcoxaJointLimitB;
    /** lower limit of the CTr joints, positive is down */
    double secondJointLimitD;
    /** upper limit of the CTr joints, positive is down */
    double secondJointLimitU;
    /** lower limit of the FTi joints, positive is down */
    double tebiaJointLimitD;
    /** upper limit of the FTi joints, positive is down */
    double tebiaJointLimitU;
    /**}*/


    /** preload of the foot spring */
    double footSpringPreload;
    /** upper limit of the foot spring = maximum value
     *  (negative is downwards (spring extends)) */
    double footSpringLimitU;
    /** lower limit of the foot spring = minimum value
     *  (negative is downwards (spring extends)) */
    double footSpringLimitD;

    /** maximal force of the backbone joint */
    double backPower;
    /** maximal force of the TC joint servos */
    double coxaPower;
    /** maximal force of the CTr joint servos */
    double secondPower;
    /** maximal force of the FTi joint servos */
    double tebiaPower;
    /** maximal force of the foot spring servos */
    double footPower;

    /** damping of the backbone joint servo */
    double backDamping;
    /** damping of the TC joint servos */
    double coxaDamping;
    /** damping of the CTr joint servo */
    double secondDamping;
    /** damping of the FTi joint servo */
    double tebiaDamping;
    /** damping of the foot spring servo */
    double footDamping;

    /** maximal velocity of the backbone joint servo */
    double backMaxVel;
    /** maximal velocity of the TC joint servo */
    double coxaMaxVel;
    /** maximal velocity of the CTr joint servo */
    double secondMaxVel;
    /** maximal velocity of the FTi joint servo */
    double tebiaMaxVel;
    /** maximal velocity of the foot spring servo */
    double footMaxVel;

    /**
     * @name front ultrasonic sensors
     *
     * configure the front ultrasonic sensors
     */
    /**{*/
    /** angle versus x axis */
    double usAngleX;
    /** angle versus y axis */
    double usAngleY;
    /** choose between parallel or antiparallel front ultrasonic sensors true
     *  means parallel */
    bool usParallel;
    /** range of the front ultrasonic sensors */
    double usRangeFront;
    /**}*/

    /** range of the infrared sensors at the legs */
    double irRangeLeg;

    /** path to texture for trunks and legs */
    std::string texture;
  };

}
#endif /* AmosIIConf_H_ */
