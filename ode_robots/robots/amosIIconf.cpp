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

#include "amosIIconf.h"

#include <cmath>

namespace lpzrobots {

    AmosIIConf AmosIIConf::getDefaultConf(
            double _scale,
            bool _useShoulder,
            bool _useFoot,
            bool _useBack)
    {
        AmosIIConf c;

        // use shoulder (fixed joint between legs and trunk)
        c.useShoulder       = _useShoulder;
        c.useTebiaJoints    = 0;
        //create springs at the end of the legs
        c.useFoot           = _useFoot;
        //create a joint in the back
        c.useBack           = _useBack;
        c.useWhiskers       = 0;
        c.rubberFeet        = false;
        c.useContactSensors = 0;
        c.useLocalVelSensor = 0;

        // the trunk length. this scales the whole robot! all parts' sizes,
        // masses, and forces will be adapted!!
        c.size              = 0.43*_scale;
        //trunk width
        c.width             = 7.0/43.0 *c.size;
        //trunk height
        c.height            = 6.5/43.0 * c.size;
        c.frontLength       = 12.0/43.0 * c.size;
        // length of upper and lower limb combined without the tip (will be
        // scaled by c.size)
        c.legLength         = 1.0/2.2;
        // we use as density the original trunk weight divided by the original
        // volume
        c.density           =  2.2/(0.43 * 0.07 * 0.065);

        c.trunkMass         = c.density * c.size * c.width * c.height;
        // use the original trunk to total mass ratio
        c.mass              = 5.758/2.2 * c.trunkMass;
        c.frontMass         = c.trunkMass * c.frontLength/c.size;
        // distribute the rest of the weight like this for now */
        c.shoulderMass      = (c.mass - c.trunkMass)
                              / (6*(3.0 + c.useShoulder))
                              * (20.0 - c.useFoot)
                              / 20.0;
        c.coxaMass          = c.shoulderMass;
        c.secondMass        = c.shoulderMass;
        c.tebiaMass         = c.shoulderMass;
        // foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by
        // 3 or 4)
        c.footMass          = (c.mass - c.trunkMass) / 6 * c.useFoot/20.0;

        //As real robot!!
        const double shoulderHeight_cm = 6.5;
        //shoulder height "4.5 wrong" --> correct=6.5 cm from rotating point
        c.shoulderHeight    = shoulderHeight_cm / 6.5 * c.height;

        // distance between hindlegs and middle legs
        c.legdist1          = 19.0/43.0 * c.size;
        // distance between middle legs and front legs
        c.legdist2          = 15.0/43.0 * c.size;

        // -----------------------
        // 1) Biomechanics
        // Manual setting adjustable joint positions at the body
        // -----------------------

        // amosII has a fixed but adjustable joint that decides how the legs
        // extend from the trunk. Here you can adjust these joints

        // ------------- Front legs -------------
        // angle (in rad) around vertical axis at leg-trunk fixation 0:
        // perpendicular
        // => forward/backward
        c.fLegTrunkAngleV   = 0.0;
        // angle around horizontal axis at leg-trunk fixation 0: perpendicular
        // => upward/downward
        c.fLegTrunkAngleH   = 0.0;
        // rotation of leg around own axis 0: first joint axis is vertical
        // => till
        c.fLegRotAngle      = 0.0;

        // ------------- Middle legs ----------------
        // => forward/backward
        c.mLegTrunkAngleV   = 0.0;
        // => upward/downward
        c.mLegTrunkAngleH   = 0.0;
        // => till
        c.mLegRotAngle      = 0.0;

        // ------------- Rear legs ------------------
        // => forward/backward
        c.rLegTrunkAngleV   = 0.0;
        // => upward/downward
        c.rLegTrunkAngleH   = 0.0;
        // => till
        c.rLegRotAngle      = 0.0;

        // the zero position of the joints depends on the placement of the two
        // bodies at the time of initialisation. specify in the following at which
        // angle the leg part should extend from the previous leg part (in rad).
        // this sets the 0 angle! don't use this, it is usually not necessary and
        // might be removed. it was intended to talk to ode's servo motors directly
        // instead of using lpzrobots set motor functions
        // front, negative is forward /
        c.fcoxaZero         = 0.0;
        // positive is down
        c.fsecondZero       = 0.0;
        // positive is down
        c.ftebiaZero        = 0.0;

        // be careful changing the following dimension, they may break the
        // simulation!! (they shouldn't but they do)
        const double shoulderLength_cm = 4.5;
        c.shoulderLength    = shoulderLength_cm/43.0*c.size;
        c.shoulderRadius    = .03*c.size;

        const double coxaLength_cm = 3.5;
        c.coxaLength        = coxaLength_cm/43.0*c.size;
        c.coxaRadius        = .04*c.size;

        const double secondLength_cm = 6.0;
        c.secondLength      = secondLength_cm/43.0*c.size;
        c.secondRadius      = .03*c.size;
        c.tebiaRadius       = 1.3/43.0*c.size;

        const double tebiaLength_cm = 11.5;// 3)
        c.tebiaLength       = tebiaLength_cm/43.0*c.size;

        // this determines the limit of the footspring
        c.footRange         = .2/43.0*c.size;
        c.footRadius        = 1.5/43.0*c.size;

        // -----------------------
        // 2) Joint Limits
        // Setting Max, Min of each joint with respect to real
        // -----------------------

        //Similar to real robot
        //-45 deg; downward (+) MIN
        c.backJointLimitD   = M_PI/180 * 45.0;
        // 45 deg; upward (-) MAX
        c.backJointLimitU   = -M_PI/180 * 45.0;

        // 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
        c.fcoxaJointLimitF  = -M_PI/180.0 * 70.0;
        //-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
        c.fcoxaJointLimitB  =  M_PI/180.0 * 70.0;

        //60 deg; forward (-) MAX --> normal walking range 30 deg MAX
        c.mcoxaJointLimitF  = -M_PI/180.0 * 60.0;
        //60 deg; backward (+) MIN --> normal walking range -40 deg MIN
        c.mcoxaJointLimitB  = M_PI/180 * 60.0;

        //70 deg; forward (-) MAX --> normal walking range 60 deg MAX
        c.rcoxaJointLimitF  = -M_PI/180.0 * 70.0;
        //70 deg; backward (+) MIN --> normal walking range -10 deg MIN
        c.rcoxaJointLimitB  = M_PI/180.0 * 70.0;

        // 70 deg; downward (+) MIN
        c.secondJointLimitD = M_PI/180.0 * 75.0;
        // 70 deg upward (-) MAX
        c.secondJointLimitU = -M_PI/180.0 * 75.0;

        //130 deg downward; (+) MIN
        c.tebiaJointLimitD  = M_PI/180.0 * 130.0;
        // 20 deg  downward; (+) MAX
        c.tebiaJointLimitU  = M_PI/180.0 * 20.0;


        // -----------------------
        // 3) Motors
        // Motor power and joint stiffness
        // -----------------------


        c.footSpringPreload = 8.0/43.0*c.size;
        // negative is downwards (spring extends)
        c.footSpringLimitD  = c.footSpringPreload;
        c.footSpringLimitU  = c.footSpringPreload + c.footRange;

        const double backPower_scale = 30.0;
        const double coxaPower_scale = 10.0;
        const double springstiffness = 350.0;

        // use an original radius and mass and scale original torque by their
        // new values to keep acceleration constant
        c.backPower         = backPower_scale * (1.962/(0.035*2.2))
                              * c.coxaLength * c.trunkMass;
        // torque in Nm
        c.coxaPower         = coxaPower_scale * (1.962/(0.035*2.2))
                              * c.coxaLength * c.trunkMass;
        c.secondPower       = c.coxaPower;
        c.tebiaPower        = c.coxaPower;
        // this is the spring constant. To keep  acceleration for the body
        // constant, we use the above unscaled preload of 0.08 and original
        // trunkMass to and then multiply by the new ones
        c.footPower         = (springstiffness * 0.08/2.2)
                              * c.trunkMass / c.footSpringPreload;

        c.backDamping       = 0.0;
        // Georg: no damping required for new servos
        c.coxaDamping       = 0.0;
        c.secondDamping     = 0.0;
        c.tebiaDamping      = 0.01;
        c.footDamping       = 0.05; // a spring has no damping??

        c.backMaxVel        = 1.961 * M_PI;
        // The speed calculates how it works
        c.coxaMaxVel        = 1.961 * M_PI;
        c.secondMaxVel      = 1.961 * M_PI;
        c.tebiaMaxVel       = 1.961 * M_PI;
        c.footMaxVel        = 1.961 * M_PI;

        c.whiskerLength     = 4.0/43.0 * c.size;

        c.usRangeFront      = 0.15*c.size;
        c.irRangeLeg        = 0.07*c.size;

        //Values by Dennis
        // 1 is parallel, -1 is antiparallel
        c.usParallel        = false;
        c.usAngleX          = 0.5;
        c.usAngleY          = 1;

        c.texture="textures/toy_fur3.jpg";

        std::cout << "trunkMass    " << c.trunkMass << "\n";
        std::cout << "Mass         " << c.mass << "\n";
        std::cout << "shoulderMass " << c.shoulderMass << "\n";
        std::cout << "footMass     " << c.footMass << "\n";
        std::cout << "backPower    " << c.backPower << "\n";


        return c;
    }

    //joint limits differ, density is reduced with scale
    AmosIIConf AmosIIConf::getExperimentConf(
            double _scale,
            bool _useShoulder,
            bool _useFoot,
            bool _useBack)
    {
        AmosIIConf c;

        //use shoulder (fixed joint between legs and trunk)
        c.useShoulder       = _useShoulder;
        c.useTebiaJoints    = 0;
        //create springs at the end of the legs
        c.useFoot           = _useFoot;
        //create a joint in the back
        c.useBack           = _useBack;
        c.useWhiskers       = 0;
        c.rubberFeet        = false;
        c.useContactSensors = 1;
        c.useLocalVelSensor = 1;

        // the trunk length. this scales the whole robot! all parts' sizes,
        // masses, and forces will be adapted!!
        c.size              = 0.43*_scale;
        // trunk width
        c.width             = 7.0/43.0 *c.size;
        // trunk height
        c.height            = 6.5/43.0 * c.size;
        c.frontLength       = 12.0/43.0 * c.size;
        // length of upper and lower limb combined without the tip (will be
        // scaled by c.size)
        c.legLength         = 1.0/2.2;
        // we use as density the original trunk weight divided by the original
        // volume
        c.density           =  2.2/(0.43 * 0.07 * 0.065)/(pow(_scale,3));
        c.trunkMass         = c.density * c.size * c.width * c.height;
        //use the original trunk to total mass ratio
        c.mass              = 5.758/2.2 * c.trunkMass;
        c.frontMass         = c.trunkMass * c.frontLength/c.size;
        //distribute the rest of the weight like this for now
        c.shoulderMass      = (c.mass - c.trunkMass)
                              / (6*(3.0 + c.useShoulder))
                              * (20.0 - c.useFoot)/20.0;
        c.coxaMass          = c.shoulderMass;
        c.secondMass        = c.shoulderMass;
        c.tebiaMass         = c.shoulderMass;
        // foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by
        // 3 or 4)
        c.footMass          = (c.mass - c.trunkMass) / 6 * c.useFoot/20.0;

        c.shoulderHeight    = 4.5 / 6.5 * c.height;
        //distance between hindlegs and middle legs
        c.legdist1          = 19.0/43.0 * c.size;
        //distance between middle legs and front legs
        c.legdist2          = 15.0/43.0 * c.size;

        // amosII has a fixed but adjustable joint that decides how the legs
        // extend from the trunk here you can adjust these joints

        // angle (in rad) around vertical axis at leg-trunk fixation 0:
        // perpendicular
        c.fLegTrunkAngleV   = 0.0;
        // angle around horizontal axis at leg-trunk fixation 0: perpendicular
        c.fLegTrunkAngleH   = 0.0;
        // rotation of leg around own axis 0: first joint axis is vertical
        c.fLegRotAngle      = 0.0;
        // middle legs and so on
        c.mLegTrunkAngleV   = 0.0;
        c.mLegTrunkAngleH   = 0.0;
        c.mLegRotAngle      = 0.0;
        c.rLegTrunkAngleV   = 0.0;
        c.rLegTrunkAngleH   = 0.0;
        c.rLegRotAngle      = 0.0;

        // the zero position of the joints depends on the placement of the two
        // bodies at the time of initialisation. specify in the following at
        // which angle the leg part should extend from the previous leg part
        // (in rad). this sets the 0 angle! don't use this, it is usually not
        // necessary and might be removed. it was intended to talk to ode's
        // servo motors directly instead of using lpzrobots set motor functions

        //front, negative is forward
        c.fcoxaZero         = 0.0;
        //positive is down
        c.fsecondZero       = 0.0;
        //positive is down
        c.ftebiaZero        = 0.0;

        //be careful changing the following dimension, they may break the
        // simulation!! (they shouldn't but they do)

        c.shoulderLength    = 4.5/43.0*c.size;
        c.shoulderRadius    = .03*c.size;
        c.coxaLength        = 3.5/43.0*c.size;
        c.coxaRadius        = .04*c.size;
        c.secondLength      = 6.0/43.0*c.size;
        c.secondRadius      = .03*c.size;
        c.tebiaRadius       = 1.3/43.0*c.size;
        c.tebiaLength       = 11.2/43.0*c.size;
        //this determines the limit of the footspring
        c.footRange         = .2/43.0*c.size;
        c.footRadius        = 1.5/43.0*c.size;

        c.backJointLimitD   = M_PI/4.0;
        c.backJointLimitU   = -M_PI/4.0;

        c.fcoxaJointLimitF  = -M_PI/180.0 * 50.0;
        c.fcoxaJointLimitB  = 0.0;
        c.mcoxaJointLimitF  = -M_PI/180.0 * 10.0;
        c.mcoxaJointLimitB  = M_PI/180 * 30.0;
        c.rcoxaJointLimitF  = M_PI/180.0 * 10.0;
        c.rcoxaJointLimitB  = M_PI/180.0 * 57.0;

        // angle range for vertical direction of legs (this is lower limit)
        c.secondJointLimitD = M_PI/4;
        c.secondJointLimitU = -M_PI/4;
        c.tebiaJointLimitD  = 3 * M_PI/4;
        c.tebiaJointLimitU  = M_PI/4;

        c.footSpringPreload = 8.0/43.0*c.size;
        //negative is downwards (spring extends)
        c.footSpringLimitD  = c.footSpringPreload;
        c.footSpringLimitU  = c.footSpringPreload + c.footRange;


        // use an original radius and mass and scale original torque by their
        // new values to keep acceleration constant
        c.backPower         = (1.962/(0.035*2.2))*c.coxaLength*c.trunkMass;
        //torque in Nm
        c.coxaPower         = c.backPower;
        c.secondPower       = c.coxaPower;
        c.tebiaPower        = c.coxaPower;
        // this is the spring constant to keep  acceleration for the body
        // constant, we use the above unscaled preload of 0.08 and original
        //trunkMass to and then multiply by the new ones
        c.footPower         = (150.0 * 0.08/2.2)
                              * c.trunkMass / c.footSpringPreload;
        c.backDamping       = 0.05;
        // Georg: no damping required for new servos
        c.coxaDamping       = 0.0;
        c.secondDamping     = 0.0;
        c.tebiaDamping      = 0.0;
        c.footDamping       = 0.05; // a spring has no damping??

        c.backMaxVel        = 1.961 * M_PI;
        // The speed calculates how it works
        c.coxaMaxVel        = 1.961 * M_PI;
        c.secondMaxVel      = 1.961 * M_PI;
        c.tebiaMaxVel       = 1.961 * M_PI;
        c.footMaxVel        = 1.961 * M_PI;

        c.whiskerLength     = 4.0/43.0 * c.size;

        c.usRangeFront      = 0.15*c.size;
        c.irRangeLeg        = 0.07*c.size;

        //Values by Dennis
        // 1 is parallel, -1 is antiparallel
        c.usParallel        = false;
        c.usAngleX          = 0.5;
        c.usAngleY          = 1;

        c.texture           = "textures/toy_fur3.jpg";

        std::cout << "trunkMass    " << c.trunkMass << "\n";
        std::cout << "Mass         " << c.mass << "\n";
        std::cout << "shoulderMass " << c.shoulderMass << "\n";
        std::cout << "footMass     " << c.footMass << "\n";
        std::cout << "backPower    " << c.backPower << "\n";

        return c;
    }

    AmosIIConf getFixedConf(
            double _scale,
            bool _useShoulder,
            bool _useFoot,
            bool _useBack)
    {
        AmosIIConf c;

        //use shoulder (fixed joint between legs and trunk)
        c.useShoulder       = _useShoulder;
        c.useTebiaJoints    = true;
        //create springs at the end of the legs
        c.useFoot           = _useFoot;
        //create a joint in the back
        c.useBack           = _useBack;
        c.useWhiskers       = 0;
        c.rubberFeet        = false;
        c.useContactSensors = 0;
        c.useLocalVelSensor = 0;

        // the trunk length. this scales the whole robot! all parts' sizes will
        // be adapted if this is changed!!
        c.size              = 0.43;
        // trunk width
        c.width             = 8.5/43.0 * c.size;
        // trunk height
        c.height            = 7.0/43.0 * c.size;
        c.frontLength       = 10.0/43.0;
        // length of upper and lower limb combined without the tip (will be
        // scaled by c.size)
        c.legLength         = 1.0/2.2;
        // we use as density the original trunk dimension divided by its volume
        c.density           = 2.2/(c.size * c.width * c.height);
        c.trunkMass         = c.density * c.size * c.width * c.height;
        // use the original trunk to total mass ratio
        c.mass              = 5.758/2.2 * c.trunkMass;
        c.frontMass         = c.trunkMass * c.frontLength/c.size;
        // distribute the rest of the weight like this for now
        c.shoulderMass      = (c.mass - c.trunkMass)
                              / (6*(3.0 + c.useShoulder))
                              * (20.0 - c.useFoot)/20.0;
        c.coxaMass          = c.shoulderMass;
        c.secondMass        = c.shoulderMass;
        c.tebiaMass         = c.shoulderMass;
        // foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by
        // 3 or 4)
        c.footMass          = (c.mass - c.trunkMass) / 6 * c.useFoot/20.0;
        c.shoulderHeight    = 4.5 / 6.5 * c.height;
        // distance between hindlegs and middle legs
        c.legdist1          = 19.0/43.0 * c.size;
        // distance between middle legs and front legs
        c.legdist2          = 15.0/43.0 * c.size;

        // amosII has a fixed but adjustable joint that decides how the legs
        // extend from the trunk here you can adjust these joints

        // angle (in rad) around vertical axis at leg-trunk fixation 0:
        // perpendicular
        c.fLegTrunkAngleV   = 0.0;
        // angle around horizontal axis at leg-trunk fixation 0: perpendicular
        c.fLegTrunkAngleH   = 0.0;
        // rotation of leg around own axis 0: first joint axis is vertical
        c.fLegRotAngle      = 0.0;
        // middle legs and so on
        c.mLegTrunkAngleV   = 0.0;
        c.mLegTrunkAngleH   = 0.0;
        c.mLegRotAngle      = 0.0;
        c.rLegTrunkAngleV   = 0.0;
        c.rLegTrunkAngleH   = 0.0;
        c.rLegRotAngle      = 0.0;

        // the zero position of the joints depends on the placement of the two
        // bodies at the time of initialisation. specify in the following at
        // which angle the leg part should extend from the previous leg part
        // (in rad). this sets the 0 angle! don't use this, it is usually not
        // necessary and might be removed. it was intended to talk to ode's
        // servo motors directly instead of using lpzrobots set motor functions

        // front, negative is forward
        c.fcoxaZero         = 0.0;
        // positive is down
        c.fsecondZero       = 0.0;
        // positive is down
        c.ftebiaZero        = 0.0;

        // be careful changing the following dimension, they may break the
        // simulation!! (they shouldn't but they do)

        c.shoulderLength    = 4.5/43.0*c.size;
        c.shoulderRadius    = .03*c.size;
        c.coxaLength        = 3.5/43.0*c.size;
        c.coxaRadius        = .04*c.size;
        c.secondLength      = 6.0/43.0*c.size;
        c.secondRadius      = .03*c.size;
        c.tebiaRadius       = 1.3/43.0*c.size;
        c.tebiaLength       = 11.2/43.0*c.size;
        //this determines the limit of the footspring
        c.footRange         = .1/43.0*c.size;
        c.footRadius        = 1.0/43.0*c.size;

        c.backJointLimitD   = M_PI/30;
        c.backJointLimitU   = -M_PI/30;

        c.fcoxaJointLimitF  = -M_PI/180.0 * 50.0;
        c.fcoxaJointLimitB  = 0.0;
        c.mcoxaJointLimitF  = -M_PI/180.0 * 10.0;
        c.mcoxaJointLimitB  = M_PI/180 * 30.0;
        c.rcoxaJointLimitF  = M_PI/180.0 * 10.0;
        c.rcoxaJointLimitB  = M_PI/180.0 * 57.0;

        // angle range for vertical direction of legs (this is lower limit)
        c.secondJointLimitD = M_PI/30.0;
        c.secondJointLimitU = -M_PI/30;
        c.tebiaJointLimitD  = 16.0 * M_PI/30.0;
        c.tebiaJointLimitU  = 14.0 * M_PI/30.0;

        c.footSpringPreload = 8.0/43.0*c.size;
        //negative is downwards (spring extends)
        c.footSpringLimitD  = c.footSpringPreload;
        c.footSpringLimitU  = c.footSpringPreload + c.footRange;

        // use an original radius and mass and scale original torque by their
        // new values to keep acceleration constant
        c.backPower         = (1.962/(0.35*2.2))*c.coxaLength*c.trunkMass;
        // torque in Nm
        c.coxaPower         = c.backPower;
        c.secondPower       = c.coxaPower;
        c.tebiaPower        = c.coxaPower;
        // this is the spring constant to keep  acceleration for the body
        // constant, we use the above unscaled preload of 0.08 and original
        // trunkMass to and then multiply by the new ones
        c.footPower         = (150.0 * 0.08/2.2)
                              * c.trunkMass / c.footSpringPreload;

        c.backDamping       = 0.05;
        // Georg: no damping required for new servos
        c.coxaDamping       = 0.0;
        c.secondDamping     = 0.0;
        c.tebiaDamping      = 0.01;
        c.footDamping       = 0.05; // a spring has no damping?

        c.backMaxVel        = 1.961 * M_PI;
        // The speed calculates how it works
        c.coxaMaxVel        = 1.961 * M_PI;
        c.secondMaxVel      = 1.961 * M_PI;
        c.tebiaMaxVel       = 1.961 * M_PI;
        c.footMaxVel        = 1.961 * M_PI;

        c.whiskerLength     = 4.0/43.0 * c.size;

        c.usRangeFront      = 0.15*c.size;
        c.irRangeLeg        = 0.07*c.size;

        //Values by Dennis
        // 1 is parallel, -1 is antiparallel
        c.usParallel        = false;
        c.usAngleX          = 0.5;
        c.usAngleY          = 1;


        return c;
      }

    AmosIIConf AmosIIConf::getFranksDefaultConf(
            double _scale,
            bool _useShoulder,
            bool _useFoot,
            bool _useBack)
    {
        AmosIIConf c;

        //use shoulder (fixed joint between legs and trunk)
        c.useShoulder       = _useShoulder;
        c.useTebiaJoints    = 0;
        //create springs at the end of the legs
        c.useFoot           = _useFoot;
        //create a joint in the back
        c.useBack           = _useBack;
        c.useWhiskers       = 0;
        c.rubberFeet        = false;
        c.useContactSensors = 0;
        c.useLocalVelSensor = 0;

        // the trunk length. this scales the whole robot! all parts' sizes,
        // masses, and forces will be adapted!!
        c.size              = 0.43*_scale;
        // trunk width
        c.width             = 7.0/43.0 *c.size;
        // trunk height
        c.height            = 6.5/43.0 * c.size;
        c.frontLength       = 12.0/43.0 * c.size;
        // length of upper and lower limb combined without the tip (will be
        // scaled by c.size)
        c.legLength         = 1.0/2.2;
        // we use as density the original trunk weight divided by the original
        // volume
        c.density           =  2.2/(0.43 * 0.07 * 0.065)/(pow(_scale,3));
        c.trunkMass         = c.density * c.size * c.width * c.height;
        // use the original trunk to total mass ratio
        c.mass              = 5.758/2.2 * c.trunkMass;
        c.frontMass         = c.trunkMass * c.frontLength/c.size;
        // distribute the rest of the weight like this for now
        c.shoulderMass      = (c.mass - c.trunkMass)
                              / (6*(3.0 + c.useShoulder))
                              * (20.0 - c.useFoot) / 20.0;
        c.coxaMass          = c.shoulderMass;
        c.secondMass        = c.shoulderMass;
        c.tebiaMass         = c.shoulderMass;
        // foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by
        // 3 or 4)
        c.footMass          = (c.mass - c.trunkMass) / 6 * c.useFoot/20.0;

        c.shoulderHeight    = 4.5 / 6.5 * c.height;
        // distance between hindlegs and middle legs
        c.legdist1          = 19.0/43.0 * c.size;
        // distance between middle legs and front legs
        c.legdist2          = 15.0/43.0 * c.size;

        // amosII has a fixed but adjustable joint that decides how the legs
        // extend from the trunk here you can adjust these joints

        /// angle (in rad) around vertical axis at leg-trunk fixation 0:
        // perpendicular
        c.fLegTrunkAngleV   = 0.0;
        // angle around horizontal axis at leg-trunk fixation 0: perpendicular
        c.fLegTrunkAngleH   = 0.0;
        // rotation of leg around own axis 0: first joint axis is vertical
        c.fLegRotAngle      = 0.0;
        // middle legs and so on
        c.mLegTrunkAngleV   = 0.0;
        c.mLegTrunkAngleH   = 0.0;
        c.mLegRotAngle      = 0.0;
        c.rLegTrunkAngleV   = 0.0;
        c.rLegTrunkAngleH   = 0.0;
        c.rLegRotAngle      = 0.0;

        // the zero position of the joints depends on the placement of the two
        // bodies at the time of initialisation. specify in the following at
        // which angle the leg part should extend from the previous leg part
        // (in rad). this sets the 0 angle!
        // don't use this, it is usually not necessary and might be removed. it
        // was intended to talk to ode's servo motors directly instead of using
        // lpzrobots set motor functions

        // front, negative is forward
        c.fcoxaZero         = 0.0;
        // positive is down
        c.fsecondZero       = 0.0;
        // positive is down
        c.ftebiaZero        = 0.0;

        // be careful changing the following dimension, they may break the
        // simulation!! (they shouldn't but they do)

        c.shoulderLength    = 4.5/43.0*c.size;
        c.shoulderRadius    = .03*c.size;
        c.coxaLength        = 3.5/43.0*c.size;
        c.coxaRadius        = .04*c.size;
        c.secondLength      = 6.0/43.0*c.size;
        c.secondRadius      = .03*c.size;
        c.tebiaRadius       = 1.3/43.0*c.size;
        c.tebiaLength       = 11.2/43.0*c.size;
        // this determines the limit of the footspring
        c.footRange         = .2/43.0*c.size;
        c.footRadius        = 1.5/43.0*c.size;

        c.backJointLimitD   = M_PI/4.0;
        c.backJointLimitU   = -M_PI/4.0;

        c.fcoxaJointLimitF  = -M_PI/180.0 * 50.0;
        c.fcoxaJointLimitB  = 0.0;
        c.mcoxaJointLimitF  = -M_PI/180.0 * 10.0;
        c.mcoxaJointLimitB  = M_PI/180 * 30.0;
        c.rcoxaJointLimitF  = M_PI/180.0 * 10.0;
        c.rcoxaJointLimitB  = M_PI/180.0 * 57.0;

        // angle range for vertical direction of legs (this is lower limit)
        c.secondJointLimitD = -M_PI/180.0 * 30;
        c.secondJointLimitU = -M_PI/180.0 * 75;
        c.tebiaJointLimitD  = 6.5 * M_PI/8.0;
        c.tebiaJointLimitU  = 5.5 * M_PI/8.0;

        c.footSpringPreload = 8.0/43.0*c.size;
        // negative is downwards (spring extends)
        c.footSpringLimitD  = c.footSpringPreload;
        c.footSpringLimitU  = c.footSpringPreload + c.footRange;

        // use an original radius and mass and scale original torque by their
        // new values to keep acceleration constant
        c.backPower         = (1.962/(0.035*2.2))*c.coxaLength*c.trunkMass;
        //torque in Nm
        c.coxaPower         = c.backPower;
        c.secondPower       = c.coxaPower;
        c.tebiaPower        = c.coxaPower;
        // this is the spring constant to keep  acceleration for the body
        // constant, we use the above unscaled preload of 0.08 and original
        // trunkMass to and then multiply by the new ones
        c.footPower         = (150.0 * 0.08/2.2)
                              * c.trunkMass / c.footSpringPreload;
        c.backDamping       = 0.05;
        // Georg: no damping required for new servos
        c.coxaDamping       = 0.0;
        c.secondDamping     = 0.0;
        c.tebiaDamping      = 0.01;
        c.footDamping       = 0.05; // a spring has no damping??

        c.backMaxVel        = 1.961 * M_PI;
        // The speed calculates how it works
        c.coxaMaxVel        = 1.961 * M_PI;
        c.secondMaxVel      = 1.961 * M_PI;
        c.tebiaMaxVel       = 1.961 * M_PI;
        c.footMaxVel        = 1.961 * M_PI;

        c.whiskerLength     = 4.0/43.0 * c.size;

        c.usRangeFront      = 0.15*c.size;
        c.irRangeLeg        = 0.07*c.size;

        //Values by Dennis
        // 1 is parallel, -1 is antiparallel
        c.usParallel        = -1;
        c.usAngleX          = 0.5;
        c.usAngleY          = 1;


        c.texture="textures/toy_fur3.jpg";

        std::cout << "trunkMass    " << c.trunkMass << "\n";
        std::cout << "Mass         " << c.mass << "\n";
        std::cout << "shoulderMass " << c.shoulderMass << "\n";
        std::cout << "footMass     " << c.footMass << "\n";
        std::cout << "backPower    " << c.backPower << "\n";

        return c;
    }

}

