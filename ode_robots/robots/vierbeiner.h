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
#ifndef __VIERBEINER_H
#define __VIERBEINER_H

#include <ode_robots/oderobot.h>

namespace lpzrobots {

  class Primitive;
  class Joint;
  class OneAxisServo;

  typedef struct {
  public:
    double size;       ///< scaling factor for robot (diameter of body)
    double legLength;  ///< length of the legs in units of size
    int    legNumber;  ///<  number of snake elements
    double mass;       ///< chassis mass
    double relLegmass; ///< relative overall leg mass
    double powerFactor;   ///< global factor for power parameters
    double dampingFactor; ///< global factor for damping parameters
    double hipPower; ///< maximal force for at hip joint motors
    double hipDamping; ///< damping of hio joint servos
    double hipJointLimit; ///< angle range for legs
    double kneePower;  ///< spring strength in the knees
    double kneeJointLimit; ///< angle range for knees
    double kneeDamping; ///< damping in the knees
    double anklePower;  ///< spring strength in the ankles
    double ankleDamping; ///< damping in the ankles
    bool   hippo;        ///< "dog" looks like a hippopotamus
    bool   drawstupidface;
    bool   useBigBox;   ///< use big box on back or not
    bool   legBodyCollisions; ///< legs and body collide
  } VierBeinerConf;


  /** robot that should look like a dog

     sensors/motors: 0: neck, 1: tail
                     2,3,4,5 : hip:  rh, lh, rf, lf
                     6,7,8,9 : knee: rh, lh, rf, lf
                     10,11   : ankle rh, lh
   */
  class VierBeiner : public OdeRobot {
  public:

    /**
     * constructor of VierBeiner robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration object
     */
    VierBeiner(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const VierBeinerConf& conf,
               const std::string& name);

    virtual ~VierBeiner(){};

    static VierBeinerConf getDefaultConf(){
      VierBeinerConf c;
      c.size               = 1;
      c.legNumber          = 4;
      c.legLength          = 0.6;
      c.mass               = 1;
      c.relLegmass         = 1;
      c.powerFactor        = 1;
      c.dampingFactor      = 1;
      c.hipPower           = 2; //3;
      c.hipDamping         = 0.1;
      c.kneePower          = 1; //2;
      c.kneeDamping        = 0.05;
      c.anklePower         = 0.1; //5;
      c.ankleDamping       = 0.02;
      c.hipJointLimit      = M_PI/3; // +- 60 degree
      c.kneeJointLimit     = M_PI/4; // +- 45 degree
      //      c.elasticity = 10;
      c.hippo              = false;
      c.drawstupidface     = 1;
      c.useBigBox          = true;
      c.legBodyCollisions  = false;
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

    /** returns actual sensorvalues
        @param sensors sensors scaled to [-1,1]
        @param sensornumber length of the sensor array
        @return number of actually written sensors
    */
    virtual int getSensorsIntern(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
        @param motors motors scaled to [-1,1]
        @param motornumber length of the motor array
    */
    virtual void setMotorsIntern(const double* motors, int motornumber);

    /** returns number of sensors
     */
    virtual int getSensorNumberIntern();

    /** returns number of motors
     */
    virtual int getMotorNumberIntern();

    /** this function is called in each timestep. It should perform robot-internal checks,
        like space-internal collision detection, sensor resets/update etc.
        @param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData);


    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);

    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return objects[0]; }
  protected:

    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();

    VierBeinerConf conf;
    double legmass;    // leg mass

    bool created;      // true if robot was created


    // some objects explicitly needed for ignored collision pairs
    Primitive *trunk, *headtrans, *bigboxtransform, *neck, *tail;
    // these ones are only need if a face is to be drawn
    Primitive *eye_r_trans, *eye_l_trans, *ear_l_trans,*ear_r_trans, *mouth_trans;


    std::vector <OneAxisServo*> hipservos; // motors
    std::vector <OneAxisServo*> kneeservos; // motors
    std::vector <OneAxisServo*> ankleservos; // motors
    std::vector <OneAxisServo*> headtailservos; // motors

    std::list <Primitive*> legparts; // lower leg parts (lower legs and feet) if collisions are ignored

  };

}

#endif
