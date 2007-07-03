/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.4  2007-07-03 13:00:14  martius
 *   new servo parameter, for current servo implementation
 *
 *   Revision 1.3  2007/04/03 16:37:13  der
 *   *** empty log message ***
 *
 *   Revision 1.2  2007/03/16 10:57:26  martius
 *   no elasticity, since substance support allows to make Playground soft
 *
 *   Revision 1.1  2007/02/23 09:30:13  der
 *   dog :-)
 *
 *   Revision 1.3  2007/02/21 16:08:45  der
 *   frontlegs no feet
 *   ankles are powered
 *   invisible pole (or box) in top
 *
 *   Revision 1.2  2007/02/12 13:30:40  martius
 *   dog looks allready nicer
 *
 *   Revision 1.1  2007/02/02 08:58:15  martius
 *   dog
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __VIERBEINER_H
#define __VIERBEINER_H

#include "oderobot.h"

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
    double hipPower; ///< maximal force for at hip joint motors
    double hipDamping; ///< damping of hio joint servos
    double hipJointLimit; ///< angle range for legs
    double kneePower;  ///< spring strength in the knees
    double kneeJointLimit; ///< angle range for knees
    double kneeDamping; ///< damping in the knees
    double anklePower;  ///< spring strength in the ankles
    double ankleDamping; ///< damping in the ankles
    double frictionGround; ///< friction with the ground
    //    double elasticity; ///< elasticity of collisions 0.5: soft 100: hard
  } VierBeinerConf;


  /** should look like a dog
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
      c.size       = 1;
      c.legNumber  = 4;
      c.legLength  = 0.6;
      c.mass       = 1;
      c.relLegmass = 1;
      c.hipPower = 3;
      c.hipDamping = 0.1;
      c.kneePower  = 2;
      c.kneeDamping = 0.05;
      c.anklePower  = 0.5;
      c.ankleDamping = 0.02;
      c.hipJointLimit = M_PI/2; // +- 90 degree
      c.kneeJointLimit = M_PI/4; // +- 45 degree
      //      c.elasticity = 10;
      return c;
    }

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update();


    /** sets the pose of the vehicle
	@param pose desired pose matrix
    */
    virtual void place(const osg::Matrix& pose);

    /** returns actual sensorvalues
	@param sensors sensors scaled to [-1,1] 
	@param sensornumber length of the sensor array
	@return number of actually written sensors
    */
    virtual int getSensors(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
	@param motors motors scaled to [-1,1] 
	@param motornumber length of the motor array
    */
    virtual void setMotors(const motor* motors, int motornumber);

    /** returns number of sensors
     */
    virtual int getSensorNumber();

    /** returns number of motors
     */
    virtual int getMotorNumber();
    /** checks for internal collisions and treats them. 
     *  In case of a treatment return true (collision will be ignored by other objects 
     *  and the default routine)  else false (collision is passed to other objects and 
     *  (if not treated) to the default routine).
     */
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(const GlobalData& globalData);

    
    /** The list of all parameters with there value as allocated lists.
     */
    virtual paramlist getParamList() const;
    
    virtual paramval getParam(const paramkey& key) const;;
    
    virtual bool setParam(const paramkey& key, paramval val);

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
    std::vector<Primitive*> objects;  // all the objects
    std::vector<Joint*> joints; // joints legs
    std::vector <OneAxisServo*> hipservos; // motors
    std::vector <OneAxisServo*> kneeservos; // motors
    std::vector <OneAxisServo*> ankleservos; // motors
    std::vector <OneAxisServo*> headtailservos; // motors

  };

}

#endif
