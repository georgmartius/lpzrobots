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
 *   Revision 1.1.4.12  2006-03-31 16:13:59  fhesse
 *   *** empty log message ***
 *
 *   Revision 1.1.4.11  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.1.4.10  2006/03/28 14:24:37  fhesse
 *   minor changes
 *
 *   Revision 1.1.4.9  2006/01/31 09:58:11  fhesse
 *   basically working now
 *
 *   Revision 1.1.4.8  2006/01/13 12:22:07  fhesse
 *   partially working
 *
 *   Revision 1.1.4.7  2006/01/10 16:45:53  fhesse
 *   not working osg version
 *
 *   Revision 1.1.4.6  2006/01/10 09:38:00  fhesse
 *   partially moved to osg
 *
 *   Revision 1.1.4.5  2005/12/16 16:36:05  fhesse
 *   manual control via keyboard
 *   setMotors via dJointAddSliderForce
 *
 *   Revision 1.1.4.4  2005/11/24 16:15:57  fhesse
 *   moved from main branch, sensors improved
 *
 *   Revision 1.3  2005/11/17 16:29:25  fhesse
 *   initial version
 *
 *   Revision 1.2  2005/11/15 12:36:27  fhesse
 *   muscles drawn as muscles, sphere drawn at tip of lower arm
 *
 *   Revision 1.1  2005/11/11 15:37:06  fhesse
 *   preinitial version
 *                                                                 *
 *                                                                         *
 ***************************************************************************/


#ifndef __MUSCLEDARM_H
#define __MUSCLEDARM_H

#include "oderobot.h"
#include <selforg/configurable.h>
#include "primitive.h"
#include "joint.h"

namespace lpzrobots{



#define SIDE (0.2)              /* side length of a box */
#define MASS (1.0)              /* mass of a capped cylinder */
#define includeMusclesGraphics false


  /* Enumeration of different parts and joints */
  // left, right up and down correspond to view from top, when base is on the bottom
  enum parts {base, upperArm, lowerArm, 
	      mainMuscle11, //left mainMuscle bottom part
	      mainMuscle12, //left mainMuscle top part
	      mainMuscle21, //right mainMuscle lower part
	      mainMuscle22, //right mainMuscle upper part
	      smallMuscle11, 
	      smallMuscle12, 
	      smallMuscle21, 
	      smallMuscle22, 
	      smallMuscle31, 
	      smallMuscle32, 
	      smallMuscle41, 
	      smallMuscle42, 
	      hand,
	      NUMParts};

  enum joints {HJ_BuA,    // hinge joint between base and upperArm
	       HJ_uAlA,   // hinge joint between upperArm and lowerArm

	       HJ_BmM11,  // hinge joint between base and mainMuscle11
	       HJ_lAmM12, // hinge joint between lowerArm and mainMuscle12
	       HJ_BmM21,  // hinge joint between base and mainMuscle21
	       HJ_lAmM22, // hinge joint between lowerArm and mainMuscle22

	       HJ_BsM11,  // hinge joint between base and smallMuscle11
	       HJ_uAsM12, // hinge joint between upperArm and smallMuscle12
	       HJ_BsM21,  // hinge joint between base and smallMuscle21
	       HJ_uAsM22, // hinge joint between upperArm and smallMuscle22
	       HJ_lAsM31, // hinge joint between lowerArm and smallMuscle31
	       HJ_uAsM32, // hinge joint between upperArm and smallMuscle32
	       HJ_lAsM41, // hinge joint between lowerArm and smallMuscle41
	       HJ_uAsM42, // hinge joint between upperArm and smallMuscle42
     
	       SJ_mM1, // sliderJoint between mainMuscle11 and mainMuscle12
	       SJ_mM2, // sliderJoint between mainMuscle21 and mainMuscle22

	       SJ_sM1, // slider Joint between smallMuscle11 ans smallMuscle12
	       SJ_sM2, // slider Joint between smallMuscle21 ans smallMuscle22
	       SJ_sM3, // slider Joint between smallMuscle31 ans smallMuscle32
	       SJ_sM4, // slider Joint between smallMuscle41 ans smallMuscle42
	       NUMJoints};



  typedef struct {
    bool jointAngleSensors; // choose sensors, all combinations are possible
    bool jointAngleRateSensors;
    bool muscleLengthSensors;
    bool jointActuator;  // if true, two motors at the joints are used
                         // if false, six muscles are used

  } MuscledArmConf;

  class MuscledArm : public OdeRobot, public Configurable{
  public:
  
    double force_[6];
  
    MuscledArm(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const MuscledArmConf& conf);

    static MuscledArmConf getDefaultConf(){
      MuscledArmConf conf;
      conf.jointAngleSensors=false;
      conf.jointAngleRateSensors=true;
      conf.muscleLengthSensors=false;
      conf.jointActuator=false;
      return conf;
    }

    virtual ~MuscledArm(){};


    /// update the subcomponents
    virtual void update();


    /** sets the pose of the vehicle
	@param pose desired 4x4 pose matrix
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
    virtual int getSensorNumber(){
      return sensorno;
    };

    /** returns number of motors
     */
    virtual int getMotorNumber(){
      return motorno;
    };

/*     /\** returns position of hand (=sphere at the end of lower arm)  */
/* 	@return position robot position in struct Position   */
/*     *\/ */
/*     virtual osg::Vec3 MuscledArm::getPosition(); */

    /** returns a vector with the positions of all segments of the robot
	@param poslist vector of positions (of all robot segments) 
	@return length of the list
    */
    virtual int getSegmentsPosition(vector<Position> &poslist);

    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(const GlobalData& globalData);

    /// returns the name of the object (with version number)
    virtual paramkey getName() const {return name; } 
  
    /** The list of all parameters with there value as allocated lists.
    */
    paramlist getParamList() const;
  
    virtual paramval getParam(const paramkey& key) const;
  
    virtual bool setParam(const paramkey& key, paramval val);

    virtual Primitive* getMainObject() const;

  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return object[lowerArm]; }

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 

    /** destroys vehicle and space
     */
    virtual void destroy();

    static void mycallback(void *data, dGeomID o1, dGeomID o2);

    double dBodyGetPositionAll ( dBodyID basis , int para );
    double dGeomGetPositionAll ( dGeomID basis , int para );
  
    void BodyCreate(int n, dMass m, dReal x, dReal y, dReal z, 
		    dReal qx, dReal qy, dReal qz, dReal qangle);

    MuscledArmConf conf;    

    static const int  armanzahl= 3;


    Primitive* object[NUMParts];  
    Joint* joint[NUMJoints]; 
    
    Position old_dist[NUMParts]; // used for damping

    string name;    
    paramval factorMotors;
    paramval factorSensors;
    paramval damping;
    paramval print;
    



    int segmentsno;    // number of motorsvehicle segments



    double gelenkabstand;
    double SOCKEL_LAENGE;
    double SOCKEL_BREITE;
    double SOCKEL_HOEHE; 
    double SOCKEL_MASSE;

    int sensorno;      //number of sensors
    int motorno;       // number of motors

    bool created;      // true if robot was created



    dSpaceID parentspace;
    
    int printed;

    double max_l;
    double max_r, min_l, min_r;

    double base_width;
    double base_length;
    double upperArm_width;
    double upperArm_length;
    double lowerArm_width;
    double lowerArm_length;
    double joint_offset;
    double mainMuscle_width;
    double mainMuscle_length;
    double smallMuscle_width;
    double smallMuscle_length;

  };

}
#endif
