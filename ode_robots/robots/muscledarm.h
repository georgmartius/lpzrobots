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
 *   Revision 1.3  2005-11-17 16:29:25  fhesse
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

#include "abstractrobot.h"
#include "configurable.h"



#define NUM 15
#define SIDE (0.2)              /* side length of a box */
#define MASS (1.0)              /* mass of a capped cylinder */
#define includeMusclesGraphics false


/* Enumeration of different parts and joints */
enum parts {fixedBody, upperArm, lowerArm, mainMuscle11, mainMuscle12, mainMuscle21, mainMuscle22, smallMuscle11, smallMuscle12, smallMuscle21, smallMuscle22, smallMuscle31, smallMuscle32, smallMuscle41, smallMuscle42};
enum joints {fixedJoint, hingeJointFUA, hingeJointUALA, hingeJointFM1, hingeJointFM2, hingeJointFS1, hingeJointFS2, hingeJointUAS1, hingeJointUAS2, hingeJointUAS3, hingeJointUAS4, hingeJointLAM1, hingeJointLAM2, hingeJointLAS3, hingeJointLAS4, sliderJointM1, sliderJointM2, sliderJointS1, sliderJointS2, sliderJointS3, sliderJointS4};



typedef struct {
  bool includeMuscles; /// should muscles be included?
  bool drawMuscles;    /// should muscles be included?
  bool drawSphere;     /// draw sphere at tip of lower Arm?
  bool strained;     /// arm strained or in resting position?
  bool jointAngleSensors;
  bool jointAngleRateSensors;
  bool MuscleLengthSensors;
} MuscledArmConf;



class MuscledArm : public AbstractRobot, public Configurable{
public:
  
  MuscledArm(const OdeHandle& odeHandle, const MuscledArmConf& conf);

  static MuscledArmConf getDefaultConf(){
    MuscledArmConf conf;
    conf.includeMuscles=true;
    conf.drawMuscles=true;
    conf.drawSphere=true;
    conf.strained=false;
    conf.jointAngleSensors=false;
    conf.jointAngleRateSensors=true;
    conf.MuscleLengthSensors=false;
    return conf;
  }

  virtual ~MuscledArm(){};

  /**
   * draws the vehicle
   */
  virtual void draw();

  void setTextures(int texture);


  /** sets the vehicle to position pos, sets color to c, and creates robot if necessary
      @params pos desired position of the robot in struct Position
      @param c desired color for the robot in struct Color
  */
  virtual void place(Position pos , Color *c = 0);

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

  /** returns position of robot 
      @return position robot position in struct Position  
  */
  virtual Position getPosition();

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);

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

  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  virtual void create(Position pos); 

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
  //Object km[armanzahl-1]; //Armglieder

  //  dJointID j[armanzahl+1];



    Object object[NUM];  
    dJointID joint[25]; 

    Position old_dist[NUM];

    string name;    
    paramval factorMotors;
    paramval factorSensors;
    paramval damping;
    



  int segmentsno;    // number of motorsvehicle segments



    double gelenkabstand;
    double SOCKEL_LAENGE;
    double SOCKEL_BREITE;
    double SOCKEL_HOEHE; 
    double SOCKEL_MASSE;

  //  double ARMDICKE;
  //  double ARMLAENGE;
  //  double ARMABSTAND;
  //  double ARMMASSE;
  /*
  double length;  // chassis length
  double width;  // chassis width
  double height;   // chassis height
  double radius;  // wheel radius
  double wheelthickness; // thickness of the wheels  
  double cmass;    // chassis mass
  double wmass;    // wheel mass
  */
  int sensorno;      //number of sensors
  int motorno;       // number of motors

  /*  double speed;    // 

  Position initial_pos;    // initial position of robot
  double max_force;        // maximal force for motors
  */
  bool created;      // true if robot was created

  int mainTexture;

  dSpaceID arm_space;

  int printed;

  double max_l;
  double max_r, min_l, min_r;
};

#endif
