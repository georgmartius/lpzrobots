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
 *   Revision 1.4.4.4  2006-01-03 10:01:46  fhesse
 *   moved to osg
 *
 *   Revision 1.4.4.3  2005/11/16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.4.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.4.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.4  2005/10/06 17:14:24  martius
 *   switched to stl lists
 *
 *   Revision 1.3  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.2  2005/09/20 11:17:29  fhesse
 *   smaller changes, needs clean up and comments
 *
 *   Revision 1.1  2005/07/28 10:22:55  fhesse
 *   initial version,
 *   known bugs: when calling show params an
 *   "pure virtual function called" error  happens
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __ARM2SEGM_H
#define __ARM2SEGM_H

#include "oderobot.h"
#include <selforg/configurable.h>
#include <vector>

#include "primitive.h"
#include "joint.h"

#include "angularmotor.h"
namespace lpzrobots{

  typedef struct {
    double max_force;  // maximal force for motors
    int segmentsno;    // number of segments

    double base_mass;  // mass of base
    double arm_mass;   // mass of arms

    double gelenkabstand;
    double SOCKEL_LAENGE;
    double SOCKEL_BREITE;
    double SOCKEL_HOEHE; 
    double SOCKEL_MASSE;

    double ARMDICKE;
    double ARMLAENGE;
    double ARMABSTAND;
    double ARMMASSE;
  } Arm2SegmConf;



  class Arm2Segm : public OdeRobot, public Configurable{
  public:
  
    Arm2Segm(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const Arm2SegmConf);

    virtual ~Arm2Segm(){};

  static Arm2SegmConf getDefaultConf(){
    Arm2SegmConf conf;
    conf.max_force=5;  // maximal force for motors
    conf.segmentsno=4;    // number of segments

    conf.base_mass=0.5;
    conf.arm_mass=0.1;

    conf.gelenkabstand =0.2;
    conf.SOCKEL_LAENGE= 0.4;
    conf.SOCKEL_BREITE= 0.1;
    conf.SOCKEL_HOEHE =0.4;
    conf.SOCKEL_MASSE =1;

    conf.ARMDICKE=0.2;
    conf.ARMLAENGE = 1.2;
    conf.ARMABSTAND= 0.03;
    conf.ARMMASSE = 0.001;

    return conf;
  }

    /// update the subcomponents
    virtual void update();

    /** sets the pose of the vehicle
	@params pose desired 4x4 pose matrix
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

    /** returns a vector with the positions of all segments of the robot
	@param poslist vector of positions (of all robot segments) 
	@return length of the list
    */
    virtual int getSegmentsPosition(vector<Position> &poslist);

    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const;

    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param GlobalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(const GlobalData& globalData);

    /// returns the name of the object (with version number)
    virtual paramkey getName() const {return name; } 
  
    /** The list of all parameters with there value as allocated lists.
	@param keylist,vallist will be allocated with malloc (free it after use!)
	@return length of the lists
    */
    paramlist getParamList() const;
  
    virtual paramval getParam(const paramkey& key) const;
  
    virtual bool setParam(const paramkey& key, paramval val);


  protected:

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 
 

    /** destroys vehicle and space
     */
    virtual void destroy();

    static void mycallback(void *data, dGeomID o1, dGeomID o2);



    dSpaceID parentspace;

    Arm2SegmConf conf;

    //    double old_angle[armanzahl+1];
    vector <Primitive*> objects;
    vector <Joint*> joints;
    vector <AngularMotor1Axis*> amotors;

    string name;    
    paramval speed;
    paramval factorSensors;

    int sensorno;      //number of sensors
    int motorno;       // number of motors
    
    bool created;      // true if robot was created

  };
};
#endif
